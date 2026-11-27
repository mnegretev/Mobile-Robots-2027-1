from collections import deque
import math
import wave

import numpy
import pyaudio
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
WAVE_OUTPUT_FILENAME = "/dev/shm/recorder_audio.wav"


class FasterWhisperNode(Node):
    def __init__(self):
        super().__init__("faster_whisper_node")
        self.robot_hablando = False
        self.sub_hablando = self.create_subscription(
            Bool, "/tts/hablando", self.recibir_hablando,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.model_size = "tiny"
        self.pwr_threshold = 0.0001
        self.pub_recognized = self.create_publisher(
            String, "/sp_rec/recognized", 10
        )

    def recibir_hablando(self, msg):
        if msg.data != self.robot_hablando:
            self.get_logger().info(
                "Escucha pausada: el robot habla."
                if msg.data else "Escucha habilitada."
            )
        self.robot_hablando = msg.data

    def potencia(self, data):
        muestras = numpy.frombuffer(data, dtype=numpy.int16)
        muestras = muestras.astype(numpy.float32) / 32768.0
        return float(numpy.mean(muestras ** 2))

    def spin(self):
        self.get_logger().info("Cargando Whisper tiny en CPU...")
        model = WhisperModel(
            self.model_size, device="cpu", compute_type="int8"
        )
        audio = pyaudio.PyAudio()
        stream = None
        previos = math.ceil(0.6 * RATE / CHUNK)
        silencio_final = math.ceil(1.5 * RATE / CHUNK)
        limite = math.ceil(20.0 * RATE / CHUNK)

        try:
            while rclpy.ok():
                stream = audio.open(
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK,
                )
                memoria = deque(maxlen=previos)
                self.get_logger().info(
                    "LISTO PARA ESCUCHAR. Di una frase."
                )

                while rclpy.ok():
                    data = stream.read(
                        CHUNK, exception_on_overflow=False
                    )
                    rclpy.spin_once(self, timeout_sec=0)
                    if self.robot_hablando:
                        memoria.clear()
                        continue
                    memoria.append(data)
                    if self.potencia(data) >= self.pwr_threshold:
                        break

                if not rclpy.ok():
                    break

                frames = list(memoria)
                interrumpida = False
                silencios = 0
                self.get_logger().info("Grabando tu frase...")

                while rclpy.ok() and len(frames) < limite:
                    data = stream.read(
                        CHUNK, exception_on_overflow=False
                    )
                    rclpy.spin_once(self, timeout_sec=0)
                    if self.robot_hablando:
                        interrumpida = True
                        break
                    frames.append(data)
                    if self.potencia(data) < self.pwr_threshold:
                        silencios += 1
                    else:
                        silencios = 0
                    if silencios >= silencio_final:
                        break

                # Cerrar la entrada evita acumular audio antiguo
                # mientras Whisper está procesando.
                stream.stop_stream()
                stream.close()
                stream = None

                if interrumpida:
                    continue

                with wave.open(WAVE_OUTPUT_FILENAME, "wb") as wav:
                    wav.setnchannels(CHANNELS)
                    wav.setsampwidth(audio.get_sample_size(FORMAT))
                    wav.setframerate(RATE)
                    wav.writeframes(b"".join(frames))

                if len(frames) >= limite:
                    self.get_logger().warning(
                        "Grabación demasiado larga. "
                        "No se enviará una orden incompleta."
                    )
                    continue

                self.get_logger().info(
                    "Grabación terminada. Procesando..."
                )
                segments, _ = model.transcribe(
                    WAVE_OUTPUT_FILENAME,
                    beam_size=5,
                    language="es",
                    condition_on_previous_text=False,
                )
                texto = " ".join(
                    segmento.text.strip() for segmento in segments
                ).strip()

                rclpy.spin_once(self, timeout_sec=0)
                if texto and not self.robot_hablando:
                    self.get_logger().info("RECONOCIDO: " + texto)
                    self.pub_recognized.publish(String(data=texto))
                else:
                    self.get_logger().info(
                        "No se reconocieron palabras."
                    )
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            if stream is not None:
                stream.stop_stream()
                stream.close()
            audio.terminate()


def main(args=None):
    rclpy.init(args=args)
    nodo = FasterWhisperNode()
    try:
        nodo.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
