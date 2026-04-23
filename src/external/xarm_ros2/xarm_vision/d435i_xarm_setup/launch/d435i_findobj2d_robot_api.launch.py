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
    calib_filename = LaunchConfiguration('calib_filename', default='')

    robot_type = robot_type.perform(context)
    dof = dof.perform(context)
    hw_ns = hw_ns.perform(context)
    if hw_ns == '':
        hw_ns = 'xarm' if robot_type == 'xarm' else 'ufactory'
    if robot_type == 'lite' or robot_type == 'uf850':
        dof = '6'
    calib_filename = calib_filename.perform(context)
    if calib_filename == '':
        calib_filename = '{}_rs_on_hand_calibration'.format(robot_type)
    else:
        calib_filename = PathJoinSubstitution([FindPackageShare('d435i_xarm_setup'), 'config', calib_filename])

    rs_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'publish_tf': 'false',
            'align_depth.enable': 'true',
            # 'camera_name': 'D435i',
            # 'camera_namespace': 'camera',
        }.items(),
    )
    
    start_find_obj_2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('d435i_xarm_setup'), 'launch', 'start_find_obj_2d.launch.py']))
    )

    extra_robot_api_params_path = PathJoinSubstitution([FindPackageShare('d435i_xarm_setup'), 'config', 'extra_robot_api_params.yaml'])
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'robot_type': robot_type,
            'dof': dof,
            'hw_ns': hw_ns,
            'add_gripper': 'true' if robot_type != 'lite' else 'false',
            'add_vacuum_gripper': 'true' if robot_type == 'lite' else 'false',
            'extra_robot_api_params_path': extra_robot_api_params_path
        }.items(),
    )

    rviz_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_rviz_display.launch.py'])),
        launch_arguments={
            'robot_type': robot_type,
            'dof': dof,
            'hw_ns': hw_ns,
            'add_gripper': 'true' if robot_type != 'lite' else 'false',
            'add_vacuum_gripper': 'true' if robot_type == 'lite' else 'false',
            'limited': 'false',
        }.items(),
    )

    publish_handeye_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('d435i_xarm_setup'), 'launch', 'publish_handeye_tf.launch.py'])),
        launch_arguments={
            'calib_filename': calib_filename,
        }.items(),
    )

    return [
        rs_camera_launch,
        start_find_obj_2d_launch,
        robot_driver_launch,
        rviz_display_launch,
        publish_handeye_tf_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
