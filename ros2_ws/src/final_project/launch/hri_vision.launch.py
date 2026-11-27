from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_mic_arg = DeclareLaunchArgument(
        "use_mic",
        default_value="true",
        description="Launch faster_whisper_asr microphone node",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Optional custom YOLOv8 .pt weights path",
    )
    conf_threshold_arg = DeclareLaunchArgument(
        "conf_threshold",
        default_value="0.20",
        description="YOLOv8 confidence threshold",
    )
    show_window_arg = DeclareLaunchArgument(
        "show_window",
        default_value="false",
        description="Open OpenCV window showing YOLOv8 detections",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="llama3",
        description="Local Ollama model name",
    )
    whisper_model_arg = DeclareLaunchArgument(
        "whisper_model",
        default_value="small",
        description="Faster-Whisper model size (tiny, base, small, medium)",
    )

    obj_recognizer_node = Node(
        package="obj_recognizer",
        executable="obj_recognizer",
        name="obj_recognizer",
        output="screen",
        parameters=[
            {"model_path": LaunchConfiguration("model_path")},
            {"conf_threshold": ParameterValue(LaunchConfiguration("conf_threshold"), value_type=float)},
            {"show_window": ParameterValue(LaunchConfiguration("show_window"), value_type=bool)},
        ],
    )

    tts_node = Node(
        package="text2speech",
        executable="t2s",
        name="text_to_speech_subscriber",
        output="screen",
        parameters=[
            {"volume": 1.0},
            {"length_scale": 1.0},
            {"play_audio": True},
        ],
    )

    orchestrator_node = Node(
        package="llm_planning",
        executable="ollama_planning",
        name="ollama_planning_node",
        output="screen",
        parameters=[
            {"model_name": LaunchConfiguration("ollama_model")},
        ],
    )

    stt_node = Node(
        package="speech2text",
        executable="faster_whisper_asr",
        name="faster_whisper_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mic")),
        parameters=[
            {"model_size": LaunchConfiguration("whisper_model")},
            {"device": "auto"},
            {"language": "es"},
        ],
    )

    return LaunchDescription([
        use_mic_arg,
        model_path_arg,
        conf_threshold_arg,
        show_window_arg,
        ollama_model_arg,
        whisper_model_arg,
        obj_recognizer_node,
        tts_node,
        orchestrator_node,
        stt_node,
    ])
