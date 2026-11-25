from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pure_pursuit',
        description='Controller for path following'
    )
    rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Open the navigation RViz window'
    )

    controller = LaunchConfiguration('controller')

    navig_utils_pkg  = get_package_share_directory('navig_utils')
    rviz_config_file = os.path.join(navig_utils_pkg, 'rviz', 'motion_planning.rviz')
    map_config_file  = os.path.join(navig_utils_pkg, 'maps', 'appartment.yaml')

    lira_gui_node = Node(
        name='lira_gui',
        package='lira_gui',
        executable='lira_gui_node',
        parameters=[{'use_sim_time':True}]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file,'--ros-args', '-p', 'use_sim_time:=True',],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
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
        executable="cost_map",
        name='cost_map',
        output='screen',
        parameters=[{'inflation_radius':0.2}, {'cost_radius':0.5}]
    )
    a_star_node = Node(
        package="path_planner",
        executable="a_star",
        name='a_star',
        output='screen',
        parameters=[{'diagonals':True}]
    )
    path_smoothing_node = Node(
        package="path_planner",
        executable="path_smoothing",
        name='path_smoothing',
        output='screen',
        parameters=[{'w1':0.95}, {'w2':0.05}]
    )
    pure_pursuit_node = Node(
        package="path_follower",
        executable="pure_pursuit",
        name='pure_pursuit',
        output='screen',
        parameters=[{'alpha':0.1}, {'beta':0.1}]
    )
    stanley_node = Node(
        package="path_follower",
        executable="stanley",
        name='stanley',
        output='screen',
        parameters=[{'Kd':1.0}, {'Ka':1.0}]
    )

    ik_node = Node(
        package='ik_numeric',
        executable='nr',
        name='ik_numeric',
        output='screen',
        parameters=[{'use_sim_time':True}, {'N':10000}]
    )
        
    return LaunchDescription([
        rviz_arg,
        lira_gui_node,
        rviz2_node,
        map_server_node,        
        amcl_node,
        TimerAction(
            period=8.0,
            actions=[
                nav2_util_node
            ]
        ),
        
        cost_map_node,
        a_star_node,
        path_smoothing_node,
        pure_pursuit_node,

        ik_node,

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_voice_bridge',
            output='screen',
            arguments=[
                '--ros-args', '-p',
                'config_file:=' + os.path.join(
                    get_package_share_directory('final_project'),
                    'config',
                    'camera_bridge.yaml'),
                '-p', 'use_sim_time:=True',
            ],
        ),
        Node(
            package='speech2text',
            executable='faster_whisper_asr',
            name='faster_whisper_asr',
            output='screen',
        ),
        Node(
            package='text2speech',
            executable='t2s',
            name='text_to_speech',
            output='screen',
        ),
        Node(
            package='obj_recognizer',
            executable='obj_recognizer_solved',
            name='obj_recognizer_solved',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='final_project',
            executable='final_project_solved',
            name='voice_skill_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
