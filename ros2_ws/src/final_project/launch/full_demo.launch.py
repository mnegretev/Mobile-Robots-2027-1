"""Launch the house and the spoken-command stack together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    whisper_model = LaunchConfiguration('whisper_model')
    house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('house_simul'),
                'launch',
                'house_simul.launch.py',
            )
        ),
        launch_arguments={'realsense': 'true'}.items(),
    )
    skills = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('final_project'),
                'launch',
                'final_project.launch.py',
            )
        ),
        launch_arguments={
            'use_rviz': 'false',
            'whisper_model': whisper_model,
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'whisper_model',
            default_value='base',
            description='faster-whisper size or local model directory',
        ),
        house,
        skills,
    ])
