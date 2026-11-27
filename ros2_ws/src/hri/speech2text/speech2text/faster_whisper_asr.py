#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# Speech-to-Text Node with Faster-Whisper and TTS Anti-Echo Guard
#

import time
import wave
import numpy
import pyaudio
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool, String

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"

CHUNK = 1024
FORMAT = pyaudio.paInt16
WAVE_OUTPUT_FILENAME = "/dev/shm/recorder_audio.wav"

HALLUCINATION_PHRASES = (
    "subtítulos realizados por",
    "subtitulos realizados por",
    "amara.org",
    "gracias por ver",
    "suscríbete",
    "suscribete",
)


class FasterWhisperNode(Node):
    """Continuous microphone STT node using faster-whisper with TTS echo suppression."""

    def __init__(self):
        super().__init__("faster_whisper_node")
        self.get_logger().info("INITIALIZING FASTER WHISPER NODE - " + NAME)

        self.declare_parameter("model_size", "small")
        self.declare_parameter("device", "auto")
        self.declare_parameter("language", "es")
        self.declare_parameter("pwr_threshold", 0.004)
        self.declare_parameter("silence_chunks", 25)
        self.declare_parameter("auto_calibrate", True)
        self.declare_parameter("max_record_seconds", 12.0)

        self.model_size = self.get_parameter("model_size").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.language = self.get_parameter("language").get_parameter_value().string_value
        self.pwr_threshold = self.get_parameter("pwr_threshold").get_parameter_value().double_value
        self.silence_chunks = self.get_parameter("silence_chunks").get_parameter_value().integer_value
        self.auto_calibrate = self.get_parameter("auto_calibrate").get_parameter_value().bool_value
        self.max_record_seconds = self.get_parameter("max_record_seconds").get_parameter_value().double_value

        self.tts_busy = False
        self.tts_last_end_time = 0.0

        self.pub_recognized = self.create_publisher(String, "/sp_rec/recognized", 10)
        self.sub_tts_busy = self.create_subscription(
            Bool, "/tts_busy", self.callback_tts_busy, 10
        )

    def callback_tts_busy(self, msg):
        """Track TTS playback state to avoid transcribing the robot's own voice."""
        if self.tts_busy and not msg.data:
            self.tts_last_end_time = time.monotonic()
        self.tts_busy = bool(msg.data)

    def is_tts_active(self):
        """Return True while TTS is speaking or within post-speech echo cooldown."""
        if self.tts_busy:
            return True
        return (time.monotonic() - self.tts_last_end_time) < 0.45

    def _load_whisper_model(self):
        """Load WhisperModel on CUDA when functional, falling back cleanly to CPU int8."""
        if self.device in ("auto", "cuda"):
            try:
                self.get_logger().info(f"Loading Faster-Whisper '{self.model_size}' on CUDA (float16)...")
                model = WhisperModel(self.model_size, device="cuda", compute_type="float16")
                # Force a short dummy pass so late cuBLAS/ctranslate2 errors surface here
                dummy_audio = numpy.zeros(8000, dtype=numpy.float32)
                segs, _ = model.transcribe(dummy_audio, language=self.language, vad_filter=False)
                list(segs)
                self.get_logger().info("Whisper model loaded on CUDA successfully.")
                return model
            except Exception as exc:
                self.get_logger().warn(f"CUDA Whisper unavailable ({exc}); falling back to CPU int8.")

        self.get_logger().info(f"Loading Faster-Whisper '{self.model_size}' on CPU (int8)...")
        model = WhisperModel(self.model_size, device="cpu", compute_type="int8")
        self.get_logger().info("Whisper model loaded on CPU successfully.")
        return model

    def _open_audio_stream(self, p):
        """Open PyAudio input stream trying optimal Whisper rates with automatic fallback."""
        candidates = [(1, 16000), (1, 44100), (2, 44100)]
        try:
            dev_info = p.get_default_input_device_info()
            default_ch = max(1, min(2, int(dev_info.get("maxInputChannels", 1))))
            default_rate = int(dev_info.get("defaultSampleRate", 16000))
            if (default_ch, default_rate) not in candidates:
                candidates.append((default_ch, default_rate))
        except Exception as exc:
            self.get_logger().warn(f"Could not query default input device info: {exc}")

        last_err = None
        for channels, rate in candidates:
            try:
                stream = p.open(
                    format=FORMAT,
                    channels=channels,
                    rate=rate,
                    input=True,
                    frames_per_buffer=CHUNK,
                )
                self.get_logger().info(f"Microphone opened: channels={channels}, rate={rate} Hz")
                return stream, channels, rate
            except Exception as exc:
                last_err = exc
        raise RuntimeError(f"Unable to open microphone input stream: {last_err}")

    def _calibrate_ambient_noise(self, stream, rate):
        """Sample ~0.8s of background audio to adapt energy threshold."""
        num_chunks = max(5, int(0.8 * rate / CHUNK))
        powers = []
        for _ in range(num_chunks):
            if not rclpy.ok():
                break
            data = stream.read(CHUNK, exception_on_overflow=False)
            arr = numpy.frombuffer(data, dtype=numpy.int16) / 32768.0
            powers.append(float(numpy.mean(arr ** 2)))
        if powers:
            ambient = float(numpy.median(powers))
            self.pwr_threshold = max(self.pwr_threshold, min(0.03, ambient * 3.5))
            self.get_logger().info(
                f"Ambient noise calibrated (median={ambient:.6f}, threshold={self.pwr_threshold:.6f})"
            )

    @staticmethod
    def _is_valid_transcription(text):
        """Filter out empty strings and common Whisper silence hallucinations."""
        if not text:
            return False
        alnum_count = sum(1 for c in text if c.isalnum())
        if alnum_count < 2:
            return False
        lowered = text.lower()
        for phrase in HALLUCINATION_PHRASES:
            if phrase in lowered:
                return False
        return True

    def spin(self):
        self.model = self._load_whisper_model()
        p = pyaudio.PyAudio()
        stream, channels, rate = self._open_audio_stream(p)

        try:
            if self.auto_calibrate:
                self._calibrate_ambient_noise(stream, rate)

            max_chunks = max(10, int(self.max_record_seconds * rate / CHUNK))
            min_chunks = max(3, int(0.25 * rate / CHUNK))

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                if self.is_tts_active():
                    stream.read(CHUNK, exception_on_overflow=False)
                    self.get_clock().sleep_for(Duration(seconds=0.005))
                    continue

                self.get_logger().info("Waiting for audio with enough power...")
                pwr = 0.0
                first_frame = None
                while pwr < self.pwr_threshold and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0)
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    if self.is_tts_active():
                        pwr = 0.0
                        continue
                    arr = numpy.frombuffer(data, dtype=numpy.int16) / 32768.0
                    pwr = float(numpy.mean(arr ** 2))
                    first_frame = data

                if not rclpy.ok() or first_frame is None:
                    break

                self.get_logger().info("Audio detected. Recording...")
                frames = [first_frame]
                no_audio_counter = 0
                aborted_by_tts = False

                while (
                    no_audio_counter < self.silence_chunks
                    and len(frames) < max_chunks
                    and rclpy.ok()
                ):
                    rclpy.spin_once(self, timeout_sec=0)
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    if self.is_tts_active():
                        aborted_by_tts = True
                        break
                    frames.append(data)
                    arr = numpy.frombuffer(data, dtype=numpy.int16) / 32768.0
                    pwr = float(numpy.mean(arr ** 2))
                    if pwr < self.pwr_threshold:
                        no_audio_counter += 1
                    else:
                        no_audio_counter = 0

                if aborted_by_tts or len(frames) < min_chunks or not rclpy.ok():
                    continue

                self.get_logger().info("Stopping audio recording. Transcribing...")
                with wave.open(WAVE_OUTPUT_FILENAME, "wb") as wf:
                    wf.setnchannels(channels)
                    wf.setsampwidth(p.get_sample_size(FORMAT))
                    wf.setframerate(rate)
                    wf.writeframes(b"".join(frames))

                segments, info = self.model.transcribe(
                    WAVE_OUTPUT_FILENAME,
                    beam_size=5,
                    language=self.language,
                    vad_filter=True,
                )
                collected = [seg.text.strip() for seg in segments if seg.text and seg.text.strip()]
                full_text = " ".join(collected).strip()

                if self._is_valid_transcription(full_text):
                    self.get_logger().info(
                        f"[{info.language} {info.language_probability:.2f}] Recognized: '{full_text}'"
                    )
                    self.pub_recognized.publish(String(data=full_text))
                else:
                    self.get_logger().debug(f"Discarded non-speech/hallucination: '{full_text}'")

                rclpy.spin_once(self, timeout_sec=0)
                self.get_clock().sleep_for(Duration(seconds=0.005))
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()


def main(args=None):
    rclpy.init(args=args)
    faster_whisper_node = FasterWhisperNode()
    try:
        faster_whisper_node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        faster_whisper_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
