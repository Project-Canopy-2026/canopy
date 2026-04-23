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
    robot_type = LaunchConfiguration('robot_type')
    dof = LaunchConfiguration('dof', default=6)
    hw_ns = LaunchConfiguration('hw_ns', default='')

    robot_type = robot_type.perform(context)
    dof = dof.perform(context)
    hw_ns = hw_ns.perform(context)
    if hw_ns == '':
        hw_ns = 'xarm' if robot_type == 'xarm' else 'ufactory'
    if robot_type == 'lite' or robot_type == 'uf850':
        dof = '6'

    findobj_grasp_node = Node(
        package='d435i_xarm_setup',
        executable='findobj_grasp_xarm_api',
        parameters=[{
            'robot_type': robot_type,
            'dof': int(dof),
            'hw_ns': hw_ns,
        }]
    )

    return [
        findobj_grasp_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
