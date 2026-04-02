import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# MAP_ORIGIN = [40.4431653, -79.9402844, 288.0961589] # steward used this for the Schenley park imagery map origin
MAP_ORIGIN = [40.44132949798969, -79.94451105594635, 293.0] # try this for the flagstaff hill zoomed in one

def generate_launch_description():

    lidar_cloud_topic = "/velodyne_points"
    lidar_cloud_frame = "velodyne"

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

    patchwork_ground_segmentation = Node(
        package="patchworkpp",
        executable="demo",
        name="ground_segmentation",
        output="screen",
        parameters=[
            {"cloud_topic": lidar_cloud_topic},
            {"frame_id": lidar_cloud_frame},
            {"sensor_height": 0.8},
            {"num_iter": 3},
            {"num_lpr": 20},
            {"num_min_pts": 0},
            {"th_seeds": 0.3},
            {"th_dist": 0.125},
            {"th_seeds_v": 0.25},
            {"th_dist_v": 0.9},
            {"max_r": 80.0},
            {"min_r": 1.0},
            {"uprightness_thr": 0.101},
            {"verbose": False},
            {"display_time": False},
        ],
        arguments=[lidar_cloud_topic],
    )

    vlp16_publisher = Node(
        package="vlp16_logger",
        executable="vlp16_publisher",
        name="vlp16_publisher",
        output="screen",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('localization'),
                'launch',
                'localization_bringup.launch.py'
            )
        )
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
        localization,
        patchwork_ground_segmentation,
        vlp16_publisher,
        fsm,
        plan_manager,
        occupancy_grid,
        cost_map,
        trajectory_planner,
        #demo_waypoint_follower,
        # rqt,
        # rviz,
    ])
