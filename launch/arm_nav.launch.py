import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    planting_fsm_test = Node(
        package='planting_controller',
        executable='planting_fsm_test',
        name='planting_fsm_test',
        output='screen'
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

    trajectory_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                '/home/teamj/dev/ros2_ws/src/canopy/launch/',
                'trajectory_planner.launch.py'
            )
        )
    )

    return LaunchDescription([
        manipulation,
        trajectory_planner,
        planting_fsm_test,
    ])