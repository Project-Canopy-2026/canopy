# Navigation + armless Planting


### Terminal 0 - CAN Bus Setup for Planting + Seedling dropped
``` bash
sudo ip link set can0 up type can bitrate 125000
ip link show can0   # verify
ls /dev/ttyACM*
######### Seedling dropped ############
cd ~/dev/ros2_ws
source install/setup.bash
ros2 run planting_controller planting_fsm_test
```


### Terminal 1 - Build and Launch the navigation + planting
``` bash
# 0. power and ssh into jetson
# power jetson with anker battery
# set up wifi on hotspot
ssh teamj@172.20.10.05


# 1. source and build
cd ~/dev/ros2_ws
source /opt/ros/humble/setup.bash
sudo timedatectl set-time "2026-04-06 14:30:00"
colcon build --symlink-install \
 --packages-skip-build-finished \
 --packages-skip realsense_gazebo_plugin xarm_moveit_servo xarm_gazebo
source install/setup.bash


# 2. connect to warthog via ethernet if not already connected
sudo ip addr flush dev enP8p1s0
sudo ip addr add 192.168.131.100/24 dev enx0050b6e57539
sudo ip route del default via 192.168.131.1 dev enP8p1s0


# 3. connect to gnss via ethernet
sudo ip addr add 192.168.0.10/24 dev enP8p1s0


# 4. connect to velodyne via ethernet
sudo ip addr add 192.168.1.100/24 dev enP8p1s0
sudo ip link set enP8p1s0 up


# 5. connect to arm via ethernet
sudo ip addr add 192.168.1.100/24 dev enP8p1s0
sudo ip link set enP8p1s0 up


# 6. check connections
ping -c 2 192.168.131.1
ping -c 2 192.168.0.222
ping -c 2 192.168.1.201


source install/setup.bash
ros2 launch /home/teamj/dev/ros2_ws/src/canopy/launch/nav_planting.launch.py
```


### Terminal 2 - ROS Bridge
``` bash
cd ~/dev/warthog_bridge
docker compose run --rm ros1_ros2_bridge bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```


### Terminal 3 - Send in Navigation Mode msg + Seedling Location
``` bash
ros2 topic pub --once /planning/requested_mode canopy_msgs/msg/Mode "{level: [2]}"


ros2 topic pub --once /planning/complete_plan canopy_msgs/msg/PlantingPlan "{bounds_geojson: '', seedlings: [{latitude: 40.440993, longitude: -79.946865, species_id: 'seedling_1'}]}"
```

# Arm + Planting
### Terminal 0 - CAN Bus Setup for Planting
``` bash
sudo ip link set can0 up type can bitrate 125000
ip link show can0   # verify
ls /dev/ttyACM*
######### Do Planting ############
cd ~/dev/ros2_ws
source install/setup.bash
ros2 run planting_controller planting_fsm_test
```

### Terminal 1 - Launch Arm
``` bash
ros2 launch /home/teamj/dev/ros2_ws/src/canopy/launch/arm_planting.launch.py


```

# Pot detection demo
### terminal1 
```bash
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic pub /behavior/enable_pot_detection std_msgs/msg/Bool "{data: true}"
```

### terminal 2
```bash
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic pub --once /behavior/seedling_dropped std_msgs/msg/Bool "{data: true}"
```

### terminal 3
```bash
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### terminal 4
```bash
cd /home/teamj/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 launch pot_detector pot_detection.launch.py
```

# Monitor Navigation Status
### Terminal 1 Distance to seedling
```bash
ssh teamj@172.20.10.05

cd ~/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic echo /planning/distance_to_seedling
```

### Terminal 2  /behavior/do_planting
```bash
ssh teamj@172.20.10.05

cd ~/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic echo  /behavior/do_planting
```

### Terminal 3  /planting_done
```bash
ssh teamj@172.20.10.05

cd ~/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic echo  /planting_done
```

### Terminal 4 /cmd_vel
```bash
ssh teamj@172.20.10.05

cd ~/dev/ros2_ws/src/canopy
source install/setup.bash
ros2 topic echo  /cmd_vel
``` 