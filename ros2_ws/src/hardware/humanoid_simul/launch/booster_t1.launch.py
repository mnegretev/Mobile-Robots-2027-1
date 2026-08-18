# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
# This is a modified version designed to work with the LIRA's software
# FI-UNAM, 2026

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    pkg_path = os.path.join(get_package_share_directory('humanoid_simul'))
    model_file = os.path.join(pkg_path, 'models', 'booster_t1', 't1.xml')

    return [
        Node(
            package='humanoid_simul',
            executable='humanoid_simul',
            name='humanoid_simul',
            output='screen',
            parameters=[{'model': model_file}]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
