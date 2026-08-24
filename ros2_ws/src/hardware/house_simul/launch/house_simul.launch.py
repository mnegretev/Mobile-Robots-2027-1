# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
# This is a modified version designed to work with the LIRA's software
# FI-UNAM, 2026
#

import os
import yaml
import numpy
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='base_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0.35"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    #add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)
    #add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_realsense_d435i = LaunchConfiguration('realsense', default=False)
    add_d435i_links = LaunchConfiguration('realsense', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('robot_description'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type.perform(context)
    )

    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'base_with_arm.urdf.xacro')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
            add_d435i_links=add_d435i_links,
            add_realsense_d435i=add_realsense_d435i
        )
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controllers_file)
        .to_moveit_configs()
    )
    moveit_config_dump = yaml.dump(moveit_config.to_dict())

    # robot gazebo launch
    # mbot_demo/launch/_robot_on_mbot_gazebo.launch.py
    # regions = [
    #     [-1.5, 3.0, 3.5, 4.5],
    #     [4.5, -4.7, 7.7, -1.5],
    #     [-1.8, -4.5, 0.0, -2.0],
    #     [-8.5, -0.7, -7.0, 0.5],
    #     [-8.5,-4.5, -4.7, -2.3]
    # ]
    # x1,y1,x2,y2 = regions[numpy.random.randint(len(regions))]
    # init_x = '0.0'#str((x2 - x1)*numpy.random.rand() + x1)
    # init_y = '0.0'#str((y2 - y1)*numpy.random.rand() + y1)
    # init_yaw = str(6.28*numpy.random.rand() - 3.14)
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('mbot_demo'), 'launch', '_robot_on_mbot_gz.launch.py'])),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            'show_rviz': 'true',
            'rviz_config': PathJoinSubstitution([FindPackageShare('mbot_demo'), 'rviz', 'moveit.rviz']),
            'world':'house.world',
            'init_x': '0.0',
            'init_y': '0.0',
            'init_yaw': '0.0'
        }.items(),
    )

    env_var_gz_models = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('house_simul'), 'models')# + ":" + os.path.join(xarm_descrip_pkg_path, "..")
    )

    tf_realsense_to_arm = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_realsense_to_arm',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_color_frame', 'diff_base_with_xarm/link6/cameradepth'],
    )

    return [
        env_var_gz_models,
        robot_gazebo_launch,
        tf_realsense_to_arm
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
