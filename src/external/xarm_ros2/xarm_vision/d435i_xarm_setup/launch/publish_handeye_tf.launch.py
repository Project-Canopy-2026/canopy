#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2025, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    calib_filename = LaunchConfiguration('calib_filename', default='')
    # calib_filepath: '~/.ros2/easy_handeye2/calibrations/{calib_filename}.calib'  # yaml format

    calib_filename = calib_filename.perform(context)
    if calib_filename == '':
        robot_type = LaunchConfiguration('robot_type')
        robot_type = robot_type.perform(context)
        calib_filename = '{}_rs_on_hand_calibration'.format(robot_type)
    
    handeye_publisher = Node(
        package='easy_handeye2',
        executable='handeye_publisher',
        parameters=[{
            'name': calib_filename
        }]
    )

    return [
        handeye_publisher,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
