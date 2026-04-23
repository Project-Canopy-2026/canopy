#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2025, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip')
    robot_type = LaunchConfiguration('robot_type')
    dof = LaunchConfiguration('dof', default=6)
    hw_ns = LaunchConfiguration('hw_ns', default='')
    marker_size = LaunchConfiguration('marker_size', default=0.15)
    marker_id = LaunchConfiguration('marker_id', default=398)

    robot_type = robot_type.perform(context)
    dof = dof.perform(context)
    hw_ns = hw_ns.perform(context)
    if hw_ns == '':
        hw_ns = 'xarm' if robot_type == 'xarm' else 'ufactory'
    if robot_type == 'lite' or robot_type == 'uf850':
        dof = '6'
    
    calib_filename = '{}_rs_on_hand_calibration'.format(robot_type)

    rs_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'publish_tf': 'false',
            # 'camera_name': 'D435i',
            # 'camera_namespace': 'camera',
        }.items(),
    )

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': marker_size,
            'marker_id': marker_id,
            'reference_frame': 'camera_color_optical_frame',
            'camera_frame': 'camera_color_optical_frame',
            'marker_frame': 'camera_marker',
            # 'corner_refinement': 'LINES',
        }],
        remappings=[
            ('/camera_info', '/camera/camera/color/camera_info'),
            ('/image', '/camera/camera/color/image_raw'),
            # ('/camera_info', '/camera/D435i/color/camera_info'),
            # ('/image', '/camera/D435i/color/image_raw')
        ],
    )

    robot_moveit_fake_launch = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_fake.launch.py'])),
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'dof': dof,
            'robot_ip': robot_ip,
            'robot_type': robot_type,
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
            # 'show_rviz': 'false',
            # 'add_realsense_d435i': 'true',
            # 'add_d435i_links': 'true',
        }.items(),
    )

    easy_handeye_calib_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'])),
        launch_arguments={
            'name': calib_filename,
            'calibration_type': 'eye_in_hand',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'camera_marker',
            'robot_base_frame': 'link_base',
            'robot_effector_frame': 'link_eef',
            # 'move_group_namespace': '/',
            # 'move_group': '{}{}'.format(robot_type, dof if robot_type != 'uf850' else ''),
            # 'freehand_robot_movement': 'true'
            # 'automatic_robot_movement': 'true'
        }.items(),
    )

    recognition_view = Node(
        package='image_view',
        executable='image_view',
        remappings=[('/image', '/aruco_single/result')],
        # arguments=['--ros-args --remap image:=/aruco_single/result']
    )

    return [
        rs_camera_launch,
        aruco_single,
        robot_moveit_fake_launch,
        easy_handeye_calib_launch,
        recognition_view
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
