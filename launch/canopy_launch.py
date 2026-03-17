from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # behavior_node = Node(
    #     package='behavior',
    #     executable='behavior',
    #     name='behavior'
    # )

    # costmaps_node = Node(
    #     package='costmaps',
    #     executable='costmaps',
    #     name='costmaps'
    # )

    # forest_planning_node = Node(
    #     package='forest_planning',
    #     executable='forest_planning',
    #     name='forest_planning'
    # )

    # route_planning_node = Node(
    #     package='route_planning',
    #     executable='route_planning',
    #     name='route_planning'
    # )

    # trajectory_planning_node = Node(
    #     package='trajectory_planning',
    #     executable='trajectory_planning',
    #     name='trajectory_planning'
    # )

    # lidar node
    lidar_node = Node(
        package='vlp16_logger_cpp',
        executable='vlp16_republisher',
        name='vlp16_logger_cpp'
    )

    return LaunchDescription([
        # behavior_node,
        # costmaps_node,
        # forest_planning_node,
        # route_planning_node,
        # trajectory_planning_node,
        lidar_node,
    ])
