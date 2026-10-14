#
# MOBILE ROBOTS - FI-UNAM, 2027-1 - Practica 01
# practica1.launch.py
#
# Launch "universal" para la Practica 01: levanta en un solo comando todo
# lo que las instrucciones piden correr en terminales separadas
# (simulacion, localizacion/mapa estatico, mapa de costo, el
# planificador elegido, y el logger de metricas), para cualquiera de
# los dos algoritmos.
#
# Ejemplos:
#   ros2 launch navig_utils practica1.launch.py algorithm:=a_star diagonals:=false
#   ros2 launch navig_utils practica1.launch.py algorithm:=a_star diagonals:=true cost_radius:=0.5
#   ros2 launch navig_utils practica1.launch.py algorithm:=rrt epsilon:=0.5 N:=100
#   ros2 launch navig_utils practica1.launch.py algorithm:=a_star use_gui:=false
#
# Lo que NO levanta a proposito: experiment_runner. Ese lo corres aparte
#   ros2 run path_planner experiment_runner --ros-args -p phase:=a_star
# una vez que veas que todo lo de aqui ya esta arriba y estable (dale
# unos segundos a la activacion de map_server/amcl).
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    algorithm         = LaunchConfiguration('algorithm').perform(context)
    diagonals         = LaunchConfiguration('diagonals').perform(context)
    epsilon           = LaunchConfiguration('epsilon').perform(context)
    n_attempts        = LaunchConfiguration('N').perform(context)
    inflation_radius  = LaunchConfiguration('inflation_radius').perform(context)
    cost_radius       = LaunchConfiguration('cost_radius').perform(context)
    use_gui           = LaunchConfiguration('use_gui').perform(context).lower() in ('true', '1')
    csv_path          = LaunchConfiguration('csv_path').perform(context)
    table_path        = LaunchConfiguration('table_path').perform(context)

    if algorithm not in ('a_star', 'rrt'):
        raise RuntimeError("algorithm debe ser 'a_star' o 'rrt', recibido: '" + algorithm + "'")

    navig_utils_pkg  = get_package_share_directory('navig_utils')
    rviz_config_file = os.path.join(navig_utils_pkg, 'rviz', 'motion_planning.rviz')
    map_config_file  = os.path.join(navig_utils_pkg, 'maps', 'appartment.yaml')

    house_simul_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('house_simul'), 'launch', 'house_simul.launch.py'])
        ),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_config_file}, {'use_sim_time': True}],
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'base_frame_id': 'base_link'},
            {'set_initial_pose': True},
            {'use_sim_time': True},
            {'alpha1': 0.01},
            {'alpha2': 0.01},
            {'alpha3': 0.1},
            {'alpha4': 0.1},
        ],
    )
    lifecycle_bringup_node = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='lifecycle_bringup',
        output='screen',
        arguments=['map_server', 'amcl'],
    )

    cost_map_node = Node(
        package='mapping',
        executable='cost_map',
        name='cost_map',
        output='screen',
        parameters=[{'inflation_radius': float(inflation_radius)}, {'cost_radius': float(cost_radius)}],
    )

    if algorithm == 'a_star':
        planner_node = Node(
            package='path_planner',
            executable='a_star',
            name='a_star_node',
            output='screen',
            parameters=[{'diagonals': diagonals.lower() in ('true', '1')}],
        )
    else:
        planner_node = Node(
            package='path_planner',
            executable='rrt',
            name='rrt_node',
            output='screen',
            parameters=[{'epsilon': float(epsilon)}, {'N': int(n_attempts)}],
        )

    metrics_logger_node = Node(
        package='path_planner',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[{'csv_path': csv_path}, {'table_path': table_path}],
    )

    actions = [
        house_simul_launch,
        map_server_node,
        amcl_node,
        TimerAction(period=5.0, actions=[lifecycle_bringup_node]),
        cost_map_node,
        planner_node,
        metrics_logger_node,
    ]

    if use_gui:
        actions.append(Node(
            name='lira_gui',
            package='lira_gui',
            executable='lira_gui_node',
            parameters=[{'use_sim_time': True}],
        ))
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file, '--ros-args', '-p', 'use_sim_time:=True'],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('algorithm', default_value='a_star',
                               description="Algoritmo de planeacion: 'a_star' o 'rrt'"),
        DeclareLaunchArgument('diagonals', default_value='false',
                               description='Solo aplica si algorithm:=a_star'),
        DeclareLaunchArgument('epsilon', default_value='0.5',
                               description='Solo aplica si algorithm:=rrt'),
        DeclareLaunchArgument('N', default_value='100',
                               description='Solo aplica si algorithm:=rrt (intentos maximos)'),
        DeclareLaunchArgument('inflation_radius', default_value='0.2'),
        DeclareLaunchArgument('cost_radius', default_value='0.2'),
        DeclareLaunchArgument('use_gui', default_value='true',
                               description='false para correr sin lira_gui/RViz (barridos headless)'),
        DeclareLaunchArgument('csv_path',
                               default_value=os.path.join(os.path.expanduser('~'), 'practica01_resultados.csv')),
        DeclareLaunchArgument('table_path',
                               default_value=os.path.join(os.path.expanduser('~'), 'tabla_comparativa_practica01.md')),
        OpaqueFunction(function=launch_setup),
    ])
