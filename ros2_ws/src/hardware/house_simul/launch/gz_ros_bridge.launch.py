from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    house_simul_pkg_path = get_package_share_directory('house_simul')
    gz_bridge_params_path = os.path.join(house_simul_pkg_path, 'config', 'gz_bridge.yaml')

    # xarm_descrip_pkg_path = get_package_share_directory('xarm_description')
    # xarm_controller_pkg_path = get_package_share_directory('xarm_controller')
    # xarm_controller_file = PathJoinSubstitution([xarm_controller_pkg_path, 'config', 'xarm6_controllers.yaml'])

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p',
                f'config_file:={gz_bridge_params_path}',
                '-p', 'use_sim_time:=True',
            ],
            output='screen'
        ),
        # Node(
        #     name='justina_gui',
        #     package='justina_gui',
        #     executable='justina_gui_node'
        # ),
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         Node(
        #             package='nav2_util',
        #             executable='lifecycle_bringup',
        #             name='lifecycle_bringup',
        #             output='screen',
        #             arguments=['map_server', 'amcl']
        #         )
        #     ]
        # )
    ])
