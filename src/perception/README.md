### calibrate lidar mounting height and 

Terminal 1: launch the driver 
```bash
ros2 run velodyne_driver velodyne_driver_node --ros-args --params-file /opt/ros/humble/share/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml  -p device_ip:=192.168.1.202
```
Terminal 2: run the pointcloud convertor
```bash
ros2 run velodyne_pointcloud velodyne_transform_node --ros-args --params-file /opt/ros/humble/share/velodyne_pointcloud/config/VLP16-velodyne_transform_node-params.yaml -p calibration:=/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml   
```



------
fit  68 roll -12.57 deg  pitch +23.35 deg  height 1.321 m  | inliers 25.8%  rms  1.0 cm  || mean roll -12.58±0.03  pitch +23.37±0.05  height 1.321±0.003
----------------------------
[fit  53] roll  +1.07 deg  pitch +19.27 deg  height 1.354 m  | inliers 25.3%  rms  0.8 cm  || mean roll  +0.97±0.55  pitch +19.32±0.39  height 1.355±0.011
[fit  54] roll  +1.06 deg  pitch +19.26 deg  height 1.353 m  | inliers 25.3%  rms  0.8 cm  || mean roll  +0.97±0.55  pitch +19.32±0.39  height 1.355±0.011
