#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2025, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    find_object_2d = Node(
        package='find_object_2d',
        executable='find_object_2d',
        output='screen',
        parameters=[{
            'gui': True,
            'approx_sync': True,
            'pnp': True,
            'subscribe_depth': True,
            'object_prefix': 'object',
            'objects_path': PathJoinSubstitution([FindPackageShare('d435i_xarm_setup'), 'objects']),
            'settings_path': '~/.ros/find_object_2d.ini',
        }],
        remappings=[
            ('rgb/image_rect_color', '/camera/camera/color/image_raw'),
            ('depth_registered/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth_registered/camera_info', '/camera/camera/aligned_depth_to_color/camera_info')
        ],
    )

    return [
        find_object_2d
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
