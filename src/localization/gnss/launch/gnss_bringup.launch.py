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

    gnss_driver_params = os.path.join(
        get_package_share_directory('gnss'),
        'config',
        'gnss_driver.yaml'
    )

    rtk_corrections = Node(
        package = 'gnss',
        name = 'ntrip_client',
        executable = 'rtk_corrections_node',
        parameters = [ntrip_config]
    )

    swiftnav_gnss_driver = Node(
        package = 'swiftnav_ros2_driver',
        name = 'swiftnav_gnss_driver',
        executable = 'sbp-to-ros',
        parameters = [gnss_driver_params]
    )

    return LaunchDescription([
        #rtk_corrections,
        swiftnav_gnss_driver
    ])