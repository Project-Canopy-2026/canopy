import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gnss_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gnss'),
                'launch',
                'gnss_bringup.launch.py'
            )
        )
    )
    
    navsat_transform_config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'navsat_transform.yaml'
    )

    global_ekf_config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'ekf_global.yaml'
    )

    # for debugging only!
    gps_static_tf = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "base_to_gps_static_tf",
        arguments = [
            "--x", "0.25",
            "--y", "0.42",
            "--z", "1.06",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "swiftnav-gnss"]
    )

    navsat_transform = Node(
        package='robot_localization', 
        executable='navsat_transform_node', 
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_config],
        remappings=[('imu', 'imu/data'), # input subscribers
                    ('gps/fix', 'gps/navsatfix'),
                    ('odometry/filtered', 'odometry/filtered'),         
                    # output publishers
                    #('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps')]
    )

    global_ekf_filter = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_map',
        output='screen',
        parameters=[global_ekf_config],
        remappings=[
            ('odometry/local', 'odometry/filtered'),
            ('odometry/filtered', 'odometry/global'),
        ]
    )

    return LaunchDescription([
        #gnss_bringup,
        gps_static_tf, # for debug only!
        navsat_transform,
        global_ekf_filter
    ])

# remapping:(internal name, system topic name)