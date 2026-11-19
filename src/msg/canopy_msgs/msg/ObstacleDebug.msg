# Obstacle-avoidance state of trajectory_planner, published every planner tick (10 Hz)
# on /planning/obstacle_debug for Foxglove (Indicator / Plot panels).

string CLEAR="CLEAR"                                  # nothing in the stop box, normal driving
string OBSTACLE_DETECTED="OBSTACLE DETECTED"          # stop box just blocked, robot braking to a stop
string CHECKING="CHECKING STATIC OR DYNAMIC"          # stopped, waiting to see if the obstacle moves
string DETOURING="DETOURING"                          # following a planned detour
string NO_PATH="NO PATH"                              # obstacle is static but no route around it; retrying each scan

std_msgs/Header header

string state            # one of the constants above
string detail           # e.g. "1.4 / 3.0 s", "2 / 3 clear scans", "waypoint 12 / 37"
string planner_status   # latest /diagnostics message from the planner (e.g. "E-stopped")

# Static-or-dynamic check
float32 wait_elapsed            # s since stopping for this obstacle (0 when not stopped)
float32 wait_required           # s an obstacle must persist to count as static
uint8 clear_scans               # consecutive clear scans seen while stopped
uint8 clear_scans_required      # clear scans needed to count the obstacle as gone

# Stop box (the area ahead that triggers a stop)
uint16 stop_box_cells           # occupied grid cells in the stop box
uint16 stop_box_cells_required  # cells needed to count as blocked
float32 grid_age                # s since the occupancy grid was built

# Detour progress
uint16 detour_waypoint          # index of the closest path point (0 when not detouring)
uint16 detour_length            # number of path points (0 when not detouring)
uint8 detour_exit_ticks         # consecutive ticks the "obstacle passed" check has passed
uint8 detour_exit_ticks_required
