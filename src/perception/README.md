### calibrate lidar mounting height and angles

Terminal 1: launch the driver 
```bash
ros2 run velodyne_driver velodyne_driver_node --ros-args --params-file /opt/ros/humble/share/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml  -p device_ip:=192.168.1.202
```
Terminal 2: run the pointcloud convertor
```bash
ros2 run velodyne_pointcloud velodyne_transform_node --ros-args --params-file /opt/ros/humble/share/velodyne_pointcloud/config/VLP16-velodyne_transform_node-params.yaml -p calibration:=/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml   
```
Terminal 3: run the calibration script
```bash
python /home/teamj/dev/ros2_ws/src/canopy/src/perception/scripts/lidar_ground_calib.py
```

Update the calibration result in both `launch/trajectory_planner_obstacle_avoidance.launch.py` and `launch/trajectory_planner_obstacle_avoidance.launch.py`

### Previoud lidar mount calib result from ground fitting
#### Nov-18
[fit  54] roll  +1.06 deg  pitch +19.26 deg  height 1.353 m  | inliers 25.3%  rms  0.8 cm  || mean roll  +0.97±0.55  pitch +19.32±0.39  height 1.355±0.011