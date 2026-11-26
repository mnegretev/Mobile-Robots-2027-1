import os
import shutil
import subprocess
import wave

import rclpy
from ament_index_python.packages import get_package_share_directory
from piper.config import SynthesisConfig
from piper.voice import PiperVoice
from rclpy.node import Node
from std_msgs.msg import Bool, String

WAVE_OUTPUT = "/dev/shm/tts_output.wav"


class TTSSubscriber(Node):
    """Synthesize speech with Piper and play it through PipeWire."""

    def __init__(self):
        super().__init__("text_to_speech_subscriber")
        package_path = get_package_share_directory("text2speech")
        self.model = os.path.join(package_path, "models/es_MX-claude-high.onnx")
        self.config = os.path.join(
            package_path, "models/es_MX-claude-high.onnx.json")
        self.voice = PiperVoice.load(
            model_path=self.model, config_path=self.config)
        self.syn_config = SynthesisConfig(
            volume=0.5,
            length_scale=1.0,
            noise_scale=0.667,
            noise_w_scale=0.8,
            normalize_audio=False,
        )
        self.subscription = self.create_subscription(
            String, "/tts_query", self.listener_callback, 10)
        self.pub_active = self.create_publisher(Bool, "/tts/active", 10)
        self.player = self._player_command()
        self.get_logger().info("Reproducción con %s" % " ".join(self.player))

    def _player_command(self):
        if shutil.which("pw-play"):
            return ["pw-play"]
        if shutil.which("paplay"):
            return ["paplay"]
        raise RuntimeError("No está pw-play ni paplay")

    def generate_speech(self, text):
        with wave.open(WAVE_OUTPUT, "wb") as wav_file:
            self.voice.synthesize_wav(text, wav_file)

    def _set_active(self, active):
        self.pub_active.publish(Bool(data=active))

    def listener_callback(self, msg):
        self.get_logger().info('Processing txt: "%s"' % msg.data)
        self._set_active(True)
        try:
            self.generate_speech(msg.data)
            subprocess.run(self.player + [WAVE_OUTPUT], check=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            self.get_logger().error("No pude reproducir la voz: %s" % exc)
        finally:
            self._set_active(False)


def main(args=None):
    rclpy.init(args=args)
    tts_processor = TTSSubscriber()
    rclpy.spin(tts_processor)
    tts_processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
