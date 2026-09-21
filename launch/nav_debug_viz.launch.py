"""Debug visualization for localization, seedlings, and global/local navigation.

Run alongside trajectory_planner.launch.py or trajectory_planne_obstacle_avoidance.launch.py,
and start it BEFORE publishing /planning/complete_plan:

    ros2 launch <repo>/launch/nav_debug_viz.launch.py                  # RViz2
    ros2 launch <repo>/launch/nav_debug_viz.launch.py foxglove:=true   # + Foxglove bridge (ws://<host>:8765)
    ros2 launch <repo>/launch/nav_debug_viz.launch.py rviz:=false foxglove:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Keep in sync with MAP_ORIGIN in trajectory_planner*.launch.py
MAP_ORIGIN = [40.44132949798969, -79.94451105594635, 293.0]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    foxglove = LaunchConfiguration("foxglove")

    nav_viz = Node(
        package="nav_debug_viz",
        executable="nav_viz_node",
        name="nav_viz_node",
        output="screen",
        parameters=[{"map_origin_lat_lon_alt_degrees": MAP_ORIGIN}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav_debug",
        output="screen",
        arguments=["-d", os.path.join(REPO_ROOT, "config", "nav_debug.rviz")],
        condition=IfCondition(rviz),
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
        condition=IfCondition(foxglove),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument(
            "foxglove", default_value="false", description="Start foxglove_bridge on port 8765"
        ),
        nav_viz,
        rviz_node,
        foxglove_bridge,
    ])
