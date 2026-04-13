import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    planting = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('planting_controller'),
                'launch',
                'planting_bringup.launch.py'
            )
        )
    )

    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('manipulation_pkg'),
                'launch',
                'arm_bringup.launch.py'
            )
        )
    )

    return LaunchDescription([
        planting,
        manipulation,
    ])