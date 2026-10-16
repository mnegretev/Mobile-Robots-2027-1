# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 02 (v2, herramienta nueva)
# motion_planning_experimentos.launch.py
#
# Variante SIN GUI (sin lira_gui ni rviz2) de motion_planning.launch.py,
# pensada para correr junto con experiment_runner.py: la meta ya no se da
# con clic en RViz, la publica experiment_runner directamente en /goal_pose.
#
# 'controller' SI selecciona el nodo activo (a diferencia del launch
# original, donde habia que comentar/descomentar una linea a mano):
#   ros2 launch navig_utils motion_planning_experimentos.launch.py controller:=pure_pursuit
#   ros2 launch navig_utils motion_planning_experimentos.launch.py controller:=stanley
#
# No modifica motion_planning.launch.py ni nada de practica2/.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_NAVIGATION_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', '..'))
DEFAULT_FOLDER = os.path.join(_NAVIGATION_DIR, 'practica2_v2', 'datos')


def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='pure_pursuit',
        description="Controlador activo: 'pure_pursuit' o 'stanley'")
    folder_arg = DeclareLaunchArgument(
        'folder', default_value=DEFAULT_FOLDER,
        description='Carpeta donde el controlador guarda goal_*.txt / robot_*.txt')

    controller = LaunchConfiguration('controller')
    folder = LaunchConfiguration('folder')

    navig_utils_pkg = get_package_share_directory('navig_utils')
    map_config_file = os.path.join(navig_utils_pkg, 'maps', 'appartment.yaml')

    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'yaml_filename': map_config_file}, {'use_sim_time': True}])
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[{'base_frame_id': 'base_link'}, {'set_initial_pose': True}, {'use_sim_time': True},
                    {'alpha1': 0.01}, {'alpha2': 0.01}, {'alpha3': 0.1}, {'alpha4': 0.1}])
    nav2_util_node = Node(
        package='nav2_util', executable='lifecycle_bringup', name='lifecycle_bringup', output='screen',
        arguments=['map_server', 'amcl'])
    cost_map_node = Node(
        package='mapping', executable='cost_map', name='cost_map', output='screen',
        parameters=[{'inflation_radius': 0.2}, {'cost_radius': 0.5}])
    a_star_node = Node(
        package='path_planner', executable='a_star', name='a_star', output='screen',
        parameters=[{'diagonals': True}])
    path_smoothing_node = Node(
        package='path_planner', executable='path_smoothing', name='path_smoothing', output='screen',
        parameters=[{'w1': 0.95}, {'w2': 0.05}])

    pure_pursuit_node = Node(
        package='path_follower', executable='pure_pursuit', name='pure_pursuit', output='screen',
        parameters=[{'alpha': 1.0}, {'beta': 0.4}, {'folder': folder}],
        condition=LaunchConfigurationEquals('controller', 'pure_pursuit'))
    stanley_node = Node(
        package='path_follower', executable='stanley', name='stanley', output='screen',
        parameters=[{'Kd': 1.0}, {'Ka': 1.0}, {'folder': folder}],
        condition=LaunchConfigurationEquals('controller', 'stanley'))

    return LaunchDescription([
        controller_arg,
        folder_arg,
        map_server_node,
        amcl_node,
        TimerAction(period=5.0, actions=[nav2_util_node]),
        cost_map_node,
        a_star_node,
        path_smoothing_node,
        pure_pursuit_node,
        stanley_node,
    ])
