import os
import queue
import re
import subprocess
import threading
import wave
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory
from piper.voice import PiperVoice
from piper.config import SynthesisConfig

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"
WAV_OUTPUT_PATH = "/dev/shm/tts_output.wav"


class TTSSubscriber(Node):
    """Asynchronous Piper-TTS ROS 2 node publishing /tts_busy status."""

    def __init__(self):
        super().__init__("text_to_speech_subscriber")
        self.get_logger().info("INITIALIZING TEXT TO SPEECH NODE - " + NAME)

        self.declare_parameter("volume", 1.0)
        self.declare_parameter("length_scale", 1.0)
        self.declare_parameter("play_audio", True)

        volume = self.get_parameter("volume").get_parameter_value().double_value
        length_scale = self.get_parameter("length_scale").get_parameter_value().double_value
        self.play_audio = self.get_parameter("play_audio").get_parameter_value().bool_value

        package_path = get_package_share_directory("text2speech")
        self.model = os.path.join(package_path, "models", "es_MX-claude-high.onnx")
        self.config = os.path.join(package_path, "models", "es_MX-claude-high.onnx.json")

        self.get_logger().info(f"Loading Piper voice model from: {self.model}")
        self.voice = PiperVoice.load(model_path=self.model, config_path=self.config)
        self.syn_config = SynthesisConfig(
            volume=volume,
            length_scale=length_scale,
            noise_scale=0.667,
            noise_w_scale=0.8,
            normalize_audio=False,
        )

        self.pub_busy = self.create_publisher(Bool, "/tts_busy", 10)
        self.subscription = self.create_subscription(
            String,
            "/tts_query",
            self.listener_callback,
            10,
        )

        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._playback_worker, daemon=True)
        self._worker.start()

    @staticmethod
    def clean_text(txt):
        """Strip markdown symbols and collapse whitespace for natural speech synthesis."""
        if not txt:
            return ""
        cleaned = re.sub(r"[*_`#]+", " ", txt)
        cleaned = re.sub(r"^\s*[-•]\s+", "", cleaned, flags=re.MULTILINE)
        cleaned = re.sub(r"\s+", " ", cleaned).strip()
        return cleaned

    def generate_speech(self, txt, output_path=WAV_OUTPUT_PATH):
        """Synthesize cleaned text into a WAV file using Piper."""
        clean_txt = self.clean_text(txt)
        if not clean_txt:
            return False
        with wave.open(output_path, "wb") as wav_file:
            self.voice.synthesize_wav(clean_txt, wav_file, syn_config=self.syn_config)
        return True

    def _play_wav(self, wav_path):
        """Play synthesized WAV with aplay, falling back to PyAudio if needed."""
        try:
            proc = subprocess.run(["aplay", "-q", wav_path], check=False)
            if proc.returncode == 0:
                return
            self.get_logger().warn(f"aplay exited with code {proc.returncode}; trying PyAudio fallback.")
        except Exception as exc:
            self.get_logger().warn(f"aplay failed ({exc}); trying PyAudio fallback.")

        try:
            import pyaudio

            with wave.open(wav_path, "rb") as wf:
                pa = pyaudio.PyAudio()
                stream = pa.open(
                    format=pa.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True,
                )
                chunk = wf.readframes(1024)
                while chunk:
                    stream.write(chunk)
                    chunk = wf.readframes(1024)
                stream.stop_stream()
                stream.close()
                pa.terminate()
        except Exception as exc:
            self.get_logger().error(f"Audio playback failed on both aplay and PyAudio: {exc}")

    def _playback_worker(self):
        """Process queued speech requests sequentially off the ROS callback thread."""
        while not self._stop_event.is_set():
            try:
                txt = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if txt is None:
                self._queue.task_done()
                break

            try:
                self.pub_busy.publish(Bool(data=True))
                if self.generate_speech(txt, WAV_OUTPUT_PATH) and self.play_audio:
                    self._play_wav(WAV_OUTPUT_PATH)
            except Exception as exc:
                self.get_logger().error(f"Error synthesizing/playing speech: {exc}")
            finally:
                self._queue.task_done()
                if self._queue.empty():
                    self.pub_busy.publish(Bool(data=False))

    def listener_callback(self, msg):
        """Enqueue incoming text queries for speech output."""
        if not msg.data or not msg.data.strip():
            return
        self.get_logger().info(f'Queuing TTS text: "{msg.data}"')
        self._queue.put(msg.data)

    def shutdown(self):
        """Stop playback worker cleanly."""
        self._stop_event.set()
        self._queue.put(None)
        if self._worker.is_alive():
            self._worker.join(timeout=1.5)


def main(args=None):
    rclpy.init(args=args)
    tts_processor = TTSSubscriber()
    try:
        rclpy.spin(tts_processor)
    except KeyboardInterrupt:
        pass
    finally:
        tts_processor.shutdown()
        tts_processor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
