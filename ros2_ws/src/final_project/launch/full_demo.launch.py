"""Launch the house and the spoken-command stack together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
        launch_arguments={'use_rviz': 'false'}.items(),
    )
    return LaunchDescription([house, skills])
