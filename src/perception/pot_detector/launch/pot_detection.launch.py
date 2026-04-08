import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    realsense_driver = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',
        parameters=[{
            'align_depth.enable': 'true',
            'enable_sync': 'true', # sync color & depth timestamps
        }]
    )

    pot_detector = Node(
        packages='pot_detector',
        executable='pot_detector',
        name='pot_detector',
        output='screen',
    )

    return LaunchDescription([
        realsense_driver,
        pot_detector,
    ])