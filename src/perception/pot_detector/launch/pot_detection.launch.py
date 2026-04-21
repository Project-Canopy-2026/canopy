import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                'camera_name': 'd405',
                'publish_tf': 'false',
                'align_depth.enable': 'true',
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
            }.items()
    )

    # realsense_driver = Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='camera',
    #     namespace='camera',
    #     output='screen',
    #     parameters=[{
    #         'align_depth.enable': True,
    #         'enable_sync': True, # sync color & depth timestamps
    #         'color_width': 640,
    #         'color_height': 480,
    #         'color_fps': 30,
    #         'depth_width': 640,
    #         'depth_height': 480,
    #         'depth_fps': 30,
    #     }]
    # )

    camera_name = 'd405'

    pot_detection = Node(
        package='pot_detector',
        executable='pot_detection_node',
        name='pot_detector',
        output='screen',
        remappings=[
            ('/camera/camera/color/image_raw',
             f'/camera/{camera_name}/color/image_raw'), 
            ('/camera/camera/aligned_depth_to_color/image_raw',
             f'/camera/{camera_name}/aligned_depth_to_color/image_raw'),
            ('/camera/camera/aligned_depth_to_color/camera_info',
             f'/camera/{camera_name}/aligned_depth_to_color/camera_info'),
        ],
    )

    return LaunchDescription([
        realsense_driver,
        pot_detection,
    ])