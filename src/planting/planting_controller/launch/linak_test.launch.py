from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    linak_can = Node(
        package='planting_controller',
        executable='linak_can_node',
        name='linak_can_node',
        output='screen',
    )

    # Opened in its own xterm so stdin works for interactive commands.
    linak_test = Node(
        package='planting_controller',
        executable='linak_test',
        name='linak_test',
        output='screen',
        prefix='xterm -e',
        emulate_tty=True,
    )

    return LaunchDescription([linak_can, linak_test])
