# Canopy
## Overview
This repository contains the software stack for the reforestation robot, developed based on ROS2.

This system covers:
- Environment perception
- Robot localization and navigation
- Control of the planting mechanism and the robot arm for planting seedlings s

The project is organized by **subsystem**.


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
│   └── canopy_bringup/     # store the launch file
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

Inside each subsystem, include a README.md to explain the code structure under this path.

For instance:
```text
├── perception/
│   │   ├── README.md
│   │   ├── seedling_detection/
│   │   └── object_detection/
```