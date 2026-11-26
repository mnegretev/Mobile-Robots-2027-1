import queue
import signal
import threading
import wave

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
WAVE_OUTPUT_FILENAME = "/dev/shm/recorder_audio.wav"
SILENCE_BLOCKS = 12
POWER_THRESHOLD = 0.05
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
        self.model_size = "small"
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
            if self._power(data) >= POWER_THRESHOLD:
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

    def _write_wav(self, frames, sample_width):
        handle = wave.open(WAVE_OUTPUT_FILENAME, "wb")
        handle.setnchannels(self.channels)
        handle.setsampwidth(sample_width)
        handle.setframerate(RATE)
        handle.writeframes(b"".join(frames))
        handle.close()

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
            "Creating Faster Whisper model with size " + self.model_size)
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
            model = WhisperModel(
                self.model_size, device="cpu", compute_type="int8")
            self.get_logger().info("Whisper model created")
            sample_width = audio.get_sample_size(FORMAT)
            self._set_state(LISTENING)
            while self.running and rclpy.ok():
                frames = self._capture_utterance()
                if not frames:
                    continue
                self._set_state(PROCESSING)
                try:
                    self._write_wav(frames, sample_width)
                    segments, info = model.transcribe(
                        WAVE_OUTPUT_FILENAME, beam_size=5, language="es")
                    self.get_logger().info(
                        "Detected language '%s' with probability %f" % (
                            info.language, info.language_probability))
                    for segment in segments:
                        self.get_logger().info(
                            "[%.2fs -> %.2fs] %s" % (
                                segment.start, segment.end, segment.text))
                        self.pub_recognized.publish(String(data=segment.text))
                        break
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
