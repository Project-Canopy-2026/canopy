import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ntrip_config = os.path.join(
        get_package_share_directory('gnss'),
        'config',
        'ntrip_client.yaml'
    )

    rtk_corrections = Node(
        package = 'gnss',
        executable = 'rtk_corrections_node',
        parameters = [ntrip_config]
    )

    gnss_driver = Node(
        package = 'gnss',
        executable = 'gnss_driver_node',
        parameters = 
    )

    return LaunchDescription([
        rtk_corrections,
        gnss_driver
    ])