import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    xacro_file = os.path.join(
        get_package_share_directory('manipulation_pkg'),
        'urdf',
        'xarm7_gripper_camera.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' dof:=7',
            ' robot_type:=xarm',
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
