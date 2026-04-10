import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'robot_params.yaml'
    )

    # xarm7 driver + MoveIt2
    xarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_planner'),
            'launch',
            'xarm7_planner_realmove.launch.py'
        ])),
        launch_arguments={
            'robot_ip': '192.168.1.201',
            'hw_ns': 'xarm',
        }.items(),
    )

    # gripper node
    gripper_node = Node(
        package='gripper_pkg',
        executable='gripper_node',
        name='gripper_node',
        parameters=[config],
    )

    # planner node — delayed 15s to let MoveIt2 fully start
    planner_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='manipulation_pkg',
                executable='planner_node',
                name='manipulation_planner',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        xarm_launch,
        gripper_node,
        planner_node,
    ])