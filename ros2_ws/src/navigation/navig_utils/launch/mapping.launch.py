from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mvn_planning_pkg = get_package_share_directory('motion_planning')
    rviz_config_file = os.path.join(mvn_planning_pkg, 'rviz', 'mapping.rviz')

    return LaunchDescription([
        Node(
            name='lira_gui',
            package='lira_gui',
            executable='lira_gui_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file,'--ros-args', '-p', 'use_sim_time:=True',],
        ),        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'slam_params_file': os.path.join(mvn_planning_pkg, 'config', 'mapper_params_online_async.yaml'),
            }.items(),
        )
    ])
