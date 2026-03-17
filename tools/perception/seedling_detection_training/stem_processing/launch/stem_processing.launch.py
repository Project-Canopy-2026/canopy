from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stem_processing",          # your package name
            executable="stem_processing",  # your node target name
            name="pointcloud_cropbox_node",
            output="screen",
            parameters=[{
                "min_x": -0.2,
                "max_x":  0.2,
                "min_y": -0.5,
                "max_y":  0.5,
                "min_z":  0.1,
                "max_z":  0.4,
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0]
            }]
        )
    ])
