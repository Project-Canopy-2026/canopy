import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    xacro_file = os.path.join(
        get_package_share_directory('manipulation_pkg'),
        'urdf',
        'xarm7_gripper_camera.urdf.xacro'
    )

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
            'xacro_file': PathJoinSubstitution([FindPackageShare('manipulation_pkg'), 'urdf', 'xarm7_gripper_camera.urdf.xacro']),
        }.items(),
    )

    # gripper node
    gripper_node = Node(
        package='gripper_pkg',
        executable='gripper_node',
        name='gripper_node',
        parameters=[config],
    )

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

    perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('[pot_detector]'),
                'launch',
                'pot_detection.launch.py'
            )
        )
    )

    # robot_description = ParameterValue(
    #     Command([
    #         'xacro ',
    #         xacro_file,
    #         ' dof:=7',
    #         ' robot_type:=xarm',
    #     ]),
    #     value_type=str
    # )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description
    #     }]
    # )

    return LaunchDescription([
        #robot_state_publisher,
        xarm_launch,
        gripper_node,
        planner_node,
        perception_node,
    ])
