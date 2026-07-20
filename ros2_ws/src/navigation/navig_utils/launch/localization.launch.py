from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    navig_utils_pkg  = get_package_share_directory('navig_utils')
    rviz_config_file = os.path.join(navig_utils_pkg, 'rviz', 'localization.rviz')
    map_config_file  = os.path.join(navig_utils_pkg, 'maps', 'appartment.yaml')

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
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename':map_config_file}, {'use_sim_time':True}]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_util',
                    executable='lifecycle_bringup',
                    name='lifecycle_bringup',
                    output='screen',
                    arguments=['map_server']
                )
            ]
        )
    ])
