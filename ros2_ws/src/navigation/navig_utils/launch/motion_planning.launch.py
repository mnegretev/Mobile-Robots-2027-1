from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pure_pursuit',
        description='Controller for path following'
    )

    controller = LaunchConfiguration('controller')

    navig_utils_pkg  = get_package_share_directory('navig_utils')
    rviz_config_file = os.path.join(navig_utils_pkg, 'rviz', 'motion_planning.rviz')
    map_config_file  = os.path.join(navig_utils_pkg, 'maps', 'appartment.yaml')

    lira_gui_node = Node(
        name='lira_gui',
        package='lira_gui',
        executable='lira_gui_node'
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file,'--ros-args', '-p', 'use_sim_time:=True',],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename':map_config_file}, {'use_sim_time':True}]
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'base_frame_id':'base_link'},
            {'set_initial_pose':True},
            {'use_sim_time':True},
            {'alpha1':0.01},
            {'alpha2':0.01},
            {'alpha3':0.1},
            {'alpha4':0.1}
        ]
    )
    nav2_util_node = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='lifecycle_bringup',
        output='screen',
        arguments=['map_server', 'amcl']
    )
    cost_map_node = Node(
        package="mapping",
        executable="cost_map_solved",
        name='cost_map',
        output='screen',
        parameters=[{'inflation_radius':0.2}, {'cost_radius':0.5}]
    )
    a_star_node = Node(
        package="path_planner",
        executable="a_star_solved",
        name='a_star',
        output='screen',
        parameters=[{'diagonals':True}]
    )
    path_smoothing_node = Node(
        package="path_planner",
        executable="path_smoothing_solved",
        name='path_smoothing',
        output='screen',
        parameters=[{'w1':0.95}, {'w2':0.05}]
    )
    pure_pursuit_node = Node(
        package="path_follower",
        executable="pure_pursuit_solved",
        name='pure_pursuit',
        output='screen',
        parameters=[{'alpha':0.1}, {'beta':0.1}]
    )
    stanley_node = Node(
        package="path_follower",
        executable="stanley_solved",
        name='stanley',
        output='screen',
        parameters=[{'Kd':1.0}, {'Ka':1.0}]
    )
        
    return LaunchDescription([
        lira_gui_node,
        rviz2_node,
        map_server_node,        
        amcl_node,
        TimerAction(
            period=5.0,
            actions=[
                nav2_util_node
            ]
        ),
        cost_map_node,
        a_star_node,
        path_smoothing_node,
        pure_pursuit_node
        #stanley_node
    ])
