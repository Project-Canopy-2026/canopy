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
        parameters = [gnss_driver_params],
        remappings = [('navsatfix', 'gps/navsatfix'),
                      ('gpsfix', 'gps/gpsfix'),
                      ('timereference', 'gps/timereference')]
    )

    gps_static_tf = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "base_to_gps_static_tf",
        arguments = [
            "--x", "-0.495",
            "--y", "0.0",
            "--z", "0.778",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "swiftnav-gnss"]
    )

    odom_to_base_tf_publisher = Node(
        package='gnss',
        executable='odom_to_base_tf_publisher',
        name='odom_to_base_tf_publisher',
        output='screen',
    )

    gnss_interface = Node(
        package='gnss',
        executable='gnss_interface',
        name='gnss_interface',
        output='screen',
    )

    return LaunchDescription([
        rtk_corrections,
        swiftnav_gnss_driver,
        gps_static_tf,
        #odom_to_base_tf_publisher,
        gnss_interface,
    ])