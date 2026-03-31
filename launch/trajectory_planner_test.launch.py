from launch import LaunchDescription
from launch_ros.actions import Node

# MAP_ORIGIN = [40.4431653, -79.9402844, 288.0961589] # steward used this for the Schenley park imagery map origin
MAP_ORIGIN = [40.44132949798969, -79.94451105594635, 293.0] # try this for the flagstaff hill zoomed in one


def generate_launch_description():

    trajectory_planner = Node(
        package="trajectory_planning",
        executable="planner",
        name="trajectory_planner",
        output="screen",
        parameters=[{"map_origin_lat_lon_alt_degrees": MAP_ORIGIN}],
    )

    cost_map = Node(
        package="costmaps",
        executable="cost_map_node",
        name="cost_map_node",
        output="screen",
        parameters=[{"map_origin_lat_lon_alt_degrees": MAP_ORIGIN}],
    )

    occupancy_grid = Node(
        package="costmaps",
        executable="occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
    )

    fsm = Node(
        package="behavior",
        executable="fsm",
        name="fsm",
        output="screen",
    )

    plan_manager = Node(
        package="behavior",
        executable="plan_manager",
        name="plan_manager",
        output="screen",
        parameters=[{"map_origin_lat_lon_alt_degrees": MAP_ORIGIN}],
    )

    demo_waypoint_follower = Node(
        package="route_planning",
        executable="demo_waypoint_follower",
        name="demo_waypoint_follower",
        output="screen",
        parameters=[{"map_origin_lat_lon_alt_degrees": MAP_ORIGIN}],
    )

    # rqt lets you inspect topics, plot values, and publish test messages
    # rqt = Node(
    #     package="rqt_gui",
    #     executable="rqt_gui",
    #     name="rqt",
    #     output="screen",
    # )

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", "/home/ronald/Documents/canopy/config/canopy.rviz"],
    # )

    return LaunchDescription([
        fsm,
        plan_manager,
        occupancy_grid,
        cost_map,
        trajectory_planner,
        demo_waypoint_follower,
        # rqt,
        # rviz,
    ])
