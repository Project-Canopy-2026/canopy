# Pot data collection Doc
### Terminal 1

Launch the real sense ros2 package. Collect images with 640 x 480 resolution.

```jsx
# 640 x 480
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  color_width:=640 color_height:=480 color_fps:=30 \
  depth_width:=640 depth_height:=480 depth_fps:=30
```

### Terminal2

Run the pot detection node.

```jsx
cd ~/dev/ros2_ws
colcon build --packages-select pot_detector
source install/setup.bash 
ros2 run pot_detector pot_detection_node
```

### Terminal3

Open foxglove, open “Canopy Pot Detection Result” Layout, to visualize the camera rgb, depth image, and the bbox detection result.

Run `foxglove_bridge` to support the data communication.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Terminal4: Record ros2 bag

Create a new folder with the date.

```bash
mkdir /home/teamj/Data/RosBag/Realsense/DATE
cd /home/teamj/Data/RosBag/Realsense/DATE
```

```jsx
ros2 bag record -o pot_1610 \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/extrinsics/depth_to_color \
  /tf_static 
```