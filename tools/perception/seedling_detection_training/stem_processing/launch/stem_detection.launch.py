from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="stem_processing",
                executable="stem_detection_node.py",
                name="stem_detection_node",
                output="screen",
            )
        ]
    )
