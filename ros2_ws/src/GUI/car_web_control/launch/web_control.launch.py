from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '127.0.0.1'
        }]
    )

    web_server = Node(
        package='car_web_control',
        executable='web_server',
        name='web_server',
        output='screen'
    )

    return LaunchDescription([
        rosbridge,
        web_server
    ])