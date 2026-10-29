import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

# Repo root (src/canopy), found through the symlinked install of this file.
CANOPY_ROOT = Path(os.path.realpath(__file__)).parents[4]
W200_BRIDGE_CONFIG = CANOPY_ROOT / 'config' / 'w200_bridge.yaml'
W200_BRIDGE_DDS_PROFILE = CANOPY_ROOT / 'config' / 'dds' / 'fastdds_w200_bridge.xml'


def check_ros_domain(context):
    # The stack must run in the bridge's Jetson-side domain. If it ran in the
    # W200's domain, the bridge would republish W200 topics back into the same
    # domain (duplicate TF, cmd_vel loops).
    bridge = yaml.safe_load(W200_BRIDGE_CONFIG.read_text())
    expected = str(bridge['to_domain'])
    actual = os.environ.get('ROS_DOMAIN_ID', '0')
    if actual != expected:
        raise RuntimeError(
            f"ROS_DOMAIN_ID is {actual}, but the Jetson stack must run in domain {expected} "
            f"(see {W200_BRIDGE_CONFIG}). Add 'export ROS_DOMAIN_ID={expected}' to ~/.bashrc "
            f"and open a new terminal.")
    return []


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

    # The only process that talks to the W200 (domain 0). It forwards just the
    # topics in config/w200_bridge.yaml to and from the Jetson stack (domain 1),
    # including the W200's TF onto /tf and /tf_static (replaces tf_relay).
    # Only this process uses the W200-facing DDS profile; every other node uses
    # the local-only profile from FASTRTPS_DEFAULT_PROFILES_FILE.
    w200_bridge = Node(
        package='domain_bridge',
        executable='domain_bridge',
        arguments=[str(W200_BRIDGE_CONFIG)],
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': str(W200_BRIDGE_DDS_PROFILE)},
        output='screen',
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
        OpaqueFunction(function=check_ros_domain),
        w200_bridge,
        rtk_corrections,
        swiftnav_gnss_driver,
        gps_static_tf,
        #odom_to_base_tf_publisher,
        gnss_interface,
    ])