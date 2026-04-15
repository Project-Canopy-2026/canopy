# Canopy
## Overview
This repository contains the software stack for the reforestation robot, developed based on ROS2.

This system covers:
- Robot localization and navigation
- Control of the planting mechanism and the robot arm for planting seedlings
- Environment perception

The project is organized by **subsystem**.

## SVD Test Setup

### Planting demo


### Pot detection demo
```bash
# terminal1 
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic pub /behavior/enable_pot_detection std_msgs/msg/Bool "{data: true}"

# terminal 2
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic pub --once /behavior/seedling_dropped std_msgs/msg/Bool "{data: true}"

# terminal 3
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# terminal 4
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 launch pot_detector pot_detection.launch.py
```

## Repository Structure
Below shows the repo structure.
```text
canopy/
├── src/                    # production robot code ONLY
│   ├── perception/         # camera, Lidar
│   ├── localization/       # SLAM
│   ├── navigation/         # path planning, obstacle avoidance
│   ├── planting_control/   # planting mechanism system control
│   ├── arm_control/        # robot arm control
│   └── integration/
├── tools/                  # dev-only, NOT on final robot
│   ├── perception/
│       ├── camera_data_collection/
│       └── lidar_data_collection/
│   ├── localization/
│   ├── navigation/
│   ├── planting_control/
│   ├── arm_control/
│   └── integration/
├── docs/                   # documentations
├── .gitignore                
├── README.md
└── LICENSE

```

Inside each subsystem, include a README.md.

For instance:
```text
├── perception/
│   │   ├── README.md
│   │   ├── seedling_detection/
│   │   └── object_detection/
```
