"""
Proyecto final: robot que obedece órdenes en lenguaje natural por voz.

Uso (con la simulación ya corriendo en otra terminal):
    ros2 launch final_project voice_robot.launch.py
Opciones:
    use_mic:=false   -> no lanza Whisper (para probar escribiendo en /voice_commander/text)
    use_tts:=false   -> no lanza Piper
    use_llm:=false   -> preguntas generales sin Ollama
    use_yolo:=false  -> reconocimiento solo por color
    mic_device:=HDA  -> micrófono para Whisper por nombre o número (ver la lista al arrancar)
    mic_threshold:=0.02 -> volumen mínimo para empezar a grabar
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fp_share = get_package_share_directory('final_project')
    args = [
        DeclareLaunchArgument('use_mic', default_value='true'),
        DeclareLaunchArgument('use_tts', default_value='true'),
        DeclareLaunchArgument('use_llm', default_value='true'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('require_name', default_value='false'),
        DeclareLaunchArgument('mic_device', default_value='',
                              description="Parte del nombre o número del micrófono para Whisper ('' = predeterminado)"),
        DeclareLaunchArgument('mic_threshold', default_value='0.05'),
    ]

    # Navegación (mapa, AMCL, costmap, A*, suavizado, pure pursuit), GUI, RViz e IK
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fp_share, 'launch', 'final_project.launch.py'))
    )

    speech2text = Node(
        package='speech2text', executable='faster_whisper_asr', name='faster_whisper_asr',
        output='screen', condition=IfCondition(LaunchConfiguration('use_mic')),
        parameters=[{
            'input_device': ParameterValue(LaunchConfiguration('mic_device'), value_type=str),
            'pwr_threshold': ParameterValue(LaunchConfiguration('mic_threshold'), value_type=float),
        }]
    )
    text2speech = Node(
        package='text2speech', executable='t2s', name='text2speech',
        output='screen', condition=IfCondition(LaunchConfiguration('use_tts'))
    )
    voice_commander = Node(
        package='final_project', executable='voice_commander', name='voice_commander',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_llm': LaunchConfiguration('use_llm'),
            'use_yolo': LaunchConfiguration('use_yolo'),
            'require_name': LaunchConfiguration('require_name'),
        }]
    )

    return LaunchDescription(args + [
        navigation,
        text2speech,
        speech2text,
        TimerAction(period=8.0, actions=[voice_commander]),
    ])
