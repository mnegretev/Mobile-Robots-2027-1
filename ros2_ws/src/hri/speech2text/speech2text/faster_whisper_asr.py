import pyaudio
import wave
import time
import collections
import numpy
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String, Bool

#
# Parameters for recording audio
#
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "/dev/shm/recorder_audio.wav"
PRE_ROLL_CHUNKS = 20          # ~0.5 s de audio previo, para no cortar la primera palabra
SILENCE_CHUNKS = 30           # ~0.7 s de silencio para dar por terminada la frase
MAX_RECORD_CHUNKS = int(10 * RATE / CHUNK)   # una frase no debe durar más de 10 s
SPEAKING_MARGIN = 0.8         # segundos que se sigue ignorando el audio después de que el robot habló

# Frases que Whisper "inventa" cuando escucha ruido o silencio (aprendidas de videos de YouTube)
HALLUCINATIONS = ["suscribete", "suscríbete", "amara.org", "subtitulos", "subtítulos",
                  "proximo video", "próximo vídeo", "próximo video", "gracias por ver",
                  "hasta la proxima", "hasta la próxima", "dale like", "nos vemos"]

# Vocabulario que ayuda a Whisper a reconocer las órdenes del proyecto
INITIAL_PROMPT = ("Órdenes para el robot Lira: ve a la cocina, ve a la sala, ve a la televisión, "
                  "ve al refrigerador, ve al estante, ve a la recámara, regresa, alto, saluda, "
                  "levanta el brazo, baja el brazo, estira el brazo, abre la pinza, cierra la pinza, "
                  "qué ves, busca la mostaza, toma la lata, tráeme la taza, dónde estás, qué hora es.")


class FasterWhisperNode(Node):
    def __init__(self):
        super().__init__("faster_whisper_node")
        self.get_logger().info("INITIALIZING FASTER WHISPER NODE")
        self.declare_parameter('model_size', 'small')
        self.declare_parameter('pwr_threshold', 0.05)
        self.declare_parameter('no_speech_threshold', 0.6)
        self.model_size = self.get_parameter('model_size').value
        self.pwr_threshold = self.get_parameter('pwr_threshold').value
        self.no_speech_threshold = self.get_parameter('no_speech_threshold').value
        self.robot_speaking = False
        self.last_speaking_time = 0.0
        self.pub_recognized = self.create_publisher(String, '/sp_rec/recognized', 1)
        self.sub_speaking = self.create_subscription(Bool, '/tts_speaking', self.callback_speaking, 10)

    def callback_speaking(self, msg):
        self.robot_speaking = msg.data
        self.last_speaking_time = time.time()

    def robot_is_talking(self):
        if self.robot_speaking:
            return True
        if time.time() - self.last_speaking_time < SPEAKING_MARGIN:
            return True
        return False

    def is_hallucination(self, text):
        t = text.lower()
        for h in HALLUCINATIONS:
            if h in t:
                return True
        return False

    def flush_stream(self, stream):
        # Descarta el audio que se acumuló mientras se transcribía
        available = stream.get_read_available()
        while available >= CHUNK:
            stream.read(CHUNK, exception_on_overflow=False)
            available = stream.get_read_available()

    def power(self, data):
        arr = numpy.frombuffer(data, dtype=numpy.int16)/32768.0
        return numpy.mean(arr**2)

    def spin(self):
        self.get_logger().info("Creating Faster Whisper model with size " + self.model_size)
        model = WhisperModel(self.model_size, device="cpu", compute_type="int8")
        self.get_logger().info("Whisper model created ")
        self.get_logger().info("Creating pyaudio stream... " + self.model_size)
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
        while rclpy.ok():
            pre_roll = collections.deque(maxlen=PRE_ROLL_CHUNKS)

            self.get_logger().info("Waiting for audio with enough power")
            pwr = 0.0
            data = b''
            peak = 0.0
            last_report = time.time()
            while rclpy.ok():
                data = stream.read(CHUNK, exception_on_overflow=False)
                rclpy.spin_once(self, timeout_sec=0)
                self.pwr_threshold = self.get_parameter('pwr_threshold').value
                if time.time() - last_report > 5.0:
                    # Ayuda a calibrar: muestra el volumen máximo que llegó en los últimos 5 s
                    self.get_logger().info("Mic level (max in 5 s): %.4f | threshold: %.4f | robot talking: %s" %
                                           (peak, self.pwr_threshold, str(self.robot_is_talking())))
                    peak = 0.0
                    last_report = time.time()
                if self.robot_is_talking():
                    pre_roll.clear()
                    continue
                pwr = self.power(data)
                if pwr > peak:
                    peak = pwr
                if pwr >= self.pwr_threshold:
                    break
                pre_roll.append(data)

            self.get_logger().info("Audio detected. Starting to record...")
            frames = list(pre_roll)
            frames.append(data)
            no_audio_counter = 0
            interrupted = False
            while no_audio_counter < SILENCE_CHUNKS and len(frames) < MAX_RECORD_CHUNKS and rclpy.ok():
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
                rclpy.spin_once(self, timeout_sec=0)
                if self.robot_is_talking():
                    interrupted = True
                    break
                pwr = self.power(data)
                if pwr < self.pwr_threshold:
                    no_audio_counter += 1
                else:
                    no_audio_counter = 0
            self.get_logger().info("Stopping audio recording.")
            if interrupted:
                self.get_logger().info("The robot started talking. Discarding recording.")
                continue

            wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

            segments, info = model.transcribe(WAVE_OUTPUT_FILENAME, beam_size=1, language="es",
                                              vad_filter=True, condition_on_previous_text=False,
                                              initial_prompt=INITIAL_PROMPT)
            texts = []
            for segment in segments:
                self.get_logger().info("[%.2fs -> %.2fs] %s (no_speech=%.2f)" %
                                       (segment.start, segment.end, segment.text, segment.no_speech_prob))
                if segment.no_speech_prob < self.no_speech_threshold:
                    texts.append(segment.text.strip())
            text = " ".join(texts).strip()

            # Lo que se haya grabado mientras se transcribía ya es viejo
            self.flush_stream(stream)

            if len(text) == 0:
                self.get_logger().info("Nothing recognized.")
            elif self.is_hallucination(text):
                self.get_logger().info("Ignoring Whisper hallucination: " + text)
            elif self.robot_is_talking():
                self.get_logger().info("Ignoring text recorded while the robot was talking: " + text)
            else:
                self.pub_recognized.publish(String(data=text))

            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.005))

        stream.stop_stream()
        stream.close()
        p.terminate()


def main(args=None):
    rclpy.init(args=args)
    faster_whisper_node = FasterWhisperNode()
    faster_whisper_node.spin()
    faster_whisper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
