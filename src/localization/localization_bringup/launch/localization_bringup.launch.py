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
                get_package_share_directory('gnss_bringup'),
                'launch',
                'gps_bringup.launch.py'
            )
    )
    )
    
    navsat_transform_config = os.path.join(
        get_package_share_directory('localization_bringup'),
        'config',
        'navsat_transform.yaml'
    )

    global_ekf_config = os.path.join(
        get_package_share_directory('localization_bringup'),
        'config',
        'global_ekf.yaml'
    )

    navsat_transform = Node{
        package='robot_localization', 
        executable='navsat_transform_node', 
        name='navsat_transform',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('imu', 'imu/data'), # input subscribers
                    ('gps/fix', 'gps/navsatfix'),
                    ('odometry/filtered', 'odometry/filtered'),         
                    # output publishers
                    #('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps')]
    }

    global_ekf_filter = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_map',
        output='screen',
        parameters=[global_ekf_config],
        remappings=[('odometry/filtered', 'odometry/gps')]
    )

    return LaunchDescription([
        gnss_bringup
        navsat_transform
        global_ekf_filter        
])

# remapping:(internal name, system topic name)