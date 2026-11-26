import queue
import signal
import threading
import time

import numpy
import pyaudio
import rclpy
from faster_whisper import WhisperModel
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String

CHUNK = 2048
FORMAT = pyaudio.paInt16
RATE = 48000
SILENCE_BLOCKS = 12
POWER_THRESHOLD = 0.008
PLAYBACK_COOLDOWN_S = 0.4

LISTENING = "LISTENING"
PROCESSING = "PROCESSING"
SPEAKING = "SPEAKING"

PREFERRED_INPUTS = ("pipewire", "pulse", "default")


class FasterWhisperNode(Node):
    """Capture speech with one persistent PortAudio stream."""

    def __init__(self):
        super().__init__("faster_whisper_node")
        self.get_logger().info("INITIALIZING FASTER WHISPER NODE")
        self.declare_parameter("model_size", "base")
        self.model_size = (
            self.get_parameter("model_size").get_parameter_value().string_value)
        self.pub_recognized = self.create_publisher(String, "/sp_rec/recognized", 1)
        self.sub_tts = self.create_subscription(
            Bool, "/tts/active", self._on_tts_active, 10)
        self.audio_q = queue.Queue(maxsize=64)
        self.lock = threading.Lock()
        self.state = PROCESSING
        self.tts_playing = False
        self.speak_gen = 0
        self.running = True
        self.channels = 1
        self._last_power_log = 0.0

    def _on_tts_active(self, msg):
        if msg.data:
            with self.lock:
                self.tts_playing = True
                self.speak_gen += 1
                if self.state != PROCESSING:
                    self.state = SPEAKING
                    self._drain_unlocked()
            self.get_logger().info("Estado SPEAKING")
            return
        with self.lock:
            self.tts_playing = False
            generation = self.speak_gen
            cooling = self.state == SPEAKING
        if cooling:
            timer = threading.Thread(
                target=self._cooldown, args=(generation,), daemon=True)
            timer.start()

    def _cooldown(self, generation):
        threading.Event().wait(PLAYBACK_COOLDOWN_S)
        with self.lock:
            if generation == self.speak_gen and self.state == SPEAKING:
                self.state = LISTENING
                self._drain_unlocked()
                self.get_logger().info("Estado LISTENING")

    def _set_state(self, state):
        with self.lock:
            self.state = state
            if state != LISTENING:
                self._drain_unlocked()
        self.get_logger().info("Estado " + state)

    def _drain_unlocked(self):
        while True:
            try:
                self.audio_q.get_nowait()
            except queue.Empty:
                return

    def _push_frame(self, data):
        try:
            self.audio_q.put_nowait(data)
            return
        except queue.Full:
            pass
        try:
            self.audio_q.get_nowait()
        except queue.Empty:
            return
        try:
            self.audio_q.put_nowait(data)
        except queue.Full:
            return

    def _reader_loop(self, stream):
        while self.running:
            try:
                data = stream.read(CHUNK, exception_on_overflow=False)
            except OSError:
                if not self.running:
                    return
                continue
            with self.lock:
                listening = self.state == LISTENING
            if listening:
                self._push_frame(data)

    def _next_frame(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            with self.lock:
                if self.state != LISTENING:
                    return None
            try:
                return self.audio_q.get(timeout=0.05)
            except queue.Empty:
                continue
        return None

    def _power(self, data):
        samples = numpy.frombuffer(data, dtype=numpy.int16).astype(numpy.float32)
        samples /= 32768.0
        return float(numpy.mean(samples * samples))

    def _capture_utterance(self):
        frames = []
        while self.running and rclpy.ok():
            data = self._next_frame()
            if data is None:
                return []
            power = self._power(data)
            now = time.monotonic()
            if now - self._last_power_log >= 1.0:
                self._last_power_log = now
                self.get_logger().info(
                    "Potencia del micrófono %.4f (umbral %.3f, estado %s)" % (
                        power, POWER_THRESHOLD, self.state))
            if power >= POWER_THRESHOLD:
                self.get_logger().info("Voz detectada")
                frames.append(data)
                break
        silence_blocks = 0
        while silence_blocks < SILENCE_BLOCKS and self.running and rclpy.ok():
            data = self._next_frame()
            if data is None:
                break
            frames.append(data)
            if self._power(data) < POWER_THRESHOLD:
                silence_blocks += 1
            else:
                silence_blocks = 0
        return frames

    def _to_whisper_audio(self, frames):
        """PCM 48 kHz int16 to mono float32 at 16 kHz, without PyAV."""
        pcm = numpy.frombuffer(b"".join(frames), dtype=numpy.int16)
        if self.channels == 2:
            usable = pcm.size - (pcm.size % 2)
            pcm = pcm[:usable].reshape(-1, 2).mean(axis=1)
        usable = pcm.size - (pcm.size % 3)
        pcm = pcm[:usable].reshape(-1, 3).mean(axis=1)
        audio = pcm.astype(numpy.float32) / 32768.0
        return numpy.ascontiguousarray(audio)

    def _select_input_device(self, audio):
        devices = []
        for index in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(index)
            if int(info.get("maxInputChannels", 0)) < 1:
                continue
            name = str(info.get("name", ""))
            lowered = name.lower()
            if "hw:" in lowered or "plughw:" in lowered:
                continue
            devices.append((index, name, lowered, int(info["maxInputChannels"])))
        for token in PREFERRED_INPUTS:
            for index, name, lowered, channel_count in devices:
                if token in lowered:
                    return index, name, channel_count
        try:
            info = audio.get_default_input_device_info()
        except OSError:
            info = None
        if info is not None:
            name = str(info.get("name", ""))
            lowered = name.lower()
            if "hw:" not in lowered and "plughw:" not in lowered:
                return (
                    int(info["index"]),
                    name,
                    int(info.get("maxInputChannels", 1)),
                )
        raise RuntimeError(
            "No hay un dispositivo de entrada pipewire, pulse o default")

    def _open_stream(self, audio, device_index, channel_count):
        channels = 2 if channel_count >= 2 else 1
        last_error = None
        for count in (channels, 1):
            try:
                stream = audio.open(
                    format=FORMAT,
                    channels=count,
                    rate=RATE,
                    input=True,
                    input_device_index=device_index,
                    frames_per_buffer=CHUNK,
                )
                return stream, count
            except OSError as exc:
                last_error = exc
        raise last_error

    def spin(self):
        self.get_logger().info(
            "Cargando Whisper '%s' en CPU (2 hilos). La primera vez lo descarga y puede tardar un minuto."
            % self.model_size)
        model = WhisperModel(
            self.model_size,
            device="cpu",
            compute_type="int8",
            cpu_threads=2,
            num_workers=1,
        )
        self.get_logger().info("Whisper model created")
        audio = pyaudio.PyAudio()
        stream = None
        reader = None
        try:
            device_index, device_name, channel_count = self._select_input_device(
                audio)
            stream, self.channels = self._open_stream(
                audio, device_index, channel_count)
            self.get_logger().info(
                "Micrófono %s a %d Hz, %d canales, buffer %d" % (
                    device_name, RATE, self.channels, CHUNK))
            reader = threading.Thread(
                target=self._reader_loop, args=(stream,), daemon=True)
            reader.start()
            self._set_state(LISTENING)
            while self.running and rclpy.ok():
                frames = self._capture_utterance()
                if not frames:
                    continue
                self._set_state(PROCESSING)
                try:
                    samples = self._to_whisper_audio(frames)
                    self.get_logger().info(
                        "Reconociendo %.1f s de audio" % (samples.size / 16000.0))
                    segments, info = model.transcribe(
                        samples, beam_size=1, language="es", vad_filter=False)
                    self.get_logger().info(
                        "Detected language '%s' with probability %f" % (
                            info.language, info.language_probability))
                    for segment in segments:
                        text = segment.text.strip()
                        if not text:
                            continue
                        self.get_logger().info(
                            "[%.2fs -> %.2fs] %s" % (
                                segment.start, segment.end, text))
                        self.pub_recognized.publish(String(data=text))
                        break
                except Exception as exc:
                    self.get_logger().error("No se pudo reconocer: %s" % exc)
                finally:
                    with self.lock:
                        follow = SPEAKING if self.tts_playing else LISTENING
                    self._set_state(follow)
                rclpy.spin_once(self, timeout_sec=0)
                self.get_clock().sleep_for(Duration(seconds=0.005))
        finally:
            self.running = False
            if reader is not None:
                reader.join(timeout=2.0)
            if stream is not None:
                stream.stop_stream()
                stream.close()
            audio.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = FasterWhisperNode()

    def _stop(_signum, _frame):
        node.running = False
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        node.spin()
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
