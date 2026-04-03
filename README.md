# Canopy
## Overview
This repository contains the software stack for the reforestation robot, developed based on ROS2.

This system covers:
- Robot localization and navigation
- Control of the planting mechanism and the robot arm for planting seedlings
- Environment perception

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
│   ├── manipulation/        # robot arm control
│   └── integration/
├── tools/                  # dev-only, NOT on final robot
│   ├── perception/
│       ├── camera_data_collection/
│       └── lidar_data_collection/
│   ├── localization/
│   ├── navigation/
│   ├── planting_control/
│   ├── manipulation/
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
