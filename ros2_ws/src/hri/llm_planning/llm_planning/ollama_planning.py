import json
import math
import queue
import threading

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
import time
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from pumas_vision_msgs.srv import RecognizeObject
from tf2_ros import Buffer, TransformListener, TransformException


DESTINOS = {
    'inicio': (0.0, 0.0),
    'punto de prueba': (-4.0, 0.0),
    'recámara': (-6.547, 0.122),
    'cocina': (6.751, -2.989),
    'gym': (2.256, 4.338),
    'gimnasio': (2.256, 4.338),
    'mesa lateral': (-0.458, 3.135),
    'comedor': (7.861, 0.914),
    'sala': (2.802, -3.873),
    'baño': (-6.416, -2.963),
}

INSTRUCCIONES = """
Eres el intérprete de un robot de servicio.
Devuelve únicamente un objeto JSON.
Si el usuario solicita ir, volver o regresar a un destino conocido:
{"accion": "navegar", "destino": "inicio"}
Los destinos conocidos son "inicio", "punto de prueba", "recámara", "cocina", "gym", "gimnasio", "mesa lateral", "comedor", "sala", "baño".
Nunca inventes coordenadas ni asignes habitaciones a esos destinos.
Si pide un destino desconocido, explica que aún no está configurado.
Para preguntas generales:
{"accion": "responder", "texto": "Respuesta breve en español"}
No afirmes haber ejecutado movimientos o reconocido objetos.
Para solicitar reconocimiento con la cámara:
{"accion": "reconocer", "objeto": "silla"}
Usa esta acción cuando pidan buscar, detectar o reconocer un objeto,
o cuando pregunten qué ves o qué objetos hay delante.
Para ver cualquier objeto usa "objeto": "".
No respondas qué ves usando conocimiento general: llama al reconocimiento.
El reconocimiento observa la imagen actual; no busca por habitaciones.
Puedes mover el brazo únicamente a dos posiciones:
{"accion": "brazo", "posicion": "prueba"}
{"accion": "brazo", "posicion": "observacion"}
Usa "brazo" solo si el usuario pide explícitamente mover el brazo.
La posición prueba cambia la muñeca a -0.5 radianes.
La posición observacion cambia la muñeca a -1.2 radianes.
No puedes agarrar, levantar ni entregar objetos.
Las respuestas habladas deben tener como máximo veinte palabras.
"""


class OllamaPlanningNode(Node):
    def __init__(self):
        super().__init__("ollama_planning_node")
        self.resultados = queue.Queue()
        self.ocupado = False
        self.meta = None
        self.llegada_recibida = False
        self.historial = []

        self.pub_tts = self.create_publisher(String, "/tts_query", 10)
        self.pub_meta = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.sub_voz = self.create_subscription(
            String, "/sp_rec/recognized", self.recibir_voz, 10
        )
        self.sub_llegada = self.create_subscription(
            Bool, "/navigation/goal_reached", self.recibir_llegada, 10
        )
        self.planificador = self.create_client(
            GetPlan, "/path_planning/plan_path"
        )
        self.articulaciones = None
        self.hora_articulaciones = 0.0
        self.sub_articulaciones = self.create_subscription(
            JointState, "/joint_states", self.recibir_articulaciones,
            qos_profile_sensor_data
        )
        self.cliente_brazo = ActionClient(
            self, FollowJointTrajectory,
            "/xarm6_traj_controller/follow_joint_trajectory"
        )
        self.vision = self.create_client(
            RecognizeObject, "/vision/recognize_object"
        )
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.actualizar)
        self.get_logger().info(
            "LISTO: voz, visión, brazo y navegación a inicio, punto de prueba, recámara, cocina y gimnasio."
        )

    def hablar(self, texto):
        self.get_logger().info(texto)
        self.pub_tts.publish(String(data=texto))

    def recibir_voz(self, msg):
        texto = msg.data.strip()
        if not texto:
            return
        if self.ocupado:
            self.get_logger().info(
                "Orden ignorada: hay una tarea en curso."
            )
            return
        self.ocupado = True
        self.get_logger().info("Orden recibida: " + texto)
        normalizado = texto.lower().translate(
            str.maketrans("áéíóúü", "aeiouu")
        )
        normalizado = " ".join(
            "".join(
                c if c.isalnum() or c.isspace() else " "
                for c in normalizado
            ).split()
        )
        preguntas_visuales = {
            "que ves", "dime que ves", "que puedes ver",
            "que objetos ves", "dime que objetos ves",
            "que hay delante", "que tienes enfrente",
        }
        if normalizado in preguntas_visuales:
            self.get_logger().info(
                "Pregunta visual: consultando la cámara."
            )
            self.reconocer("")
            return
        threading.Thread(
            target=self.consultar_ollama, args=(texto,), daemon=True
        ).start()

    def consultar_ollama(self, texto):
        try:
            mensajes = [
                {"role": "system", "content": INSTRUCCIONES}
            ] + self.historial[-8:] + [
                {"role": "user", "content": texto}
            ]
            respuesta = requests.post(
                "http://localhost:11434/api/chat",
                json={
                    "model": "qwen2.5:1.5b",
                    "messages": mensajes,
                    "stream": False,
                    "format": "json",
                    "options": {"num_ctx": 2048, "temperature": 0},
                },
                timeout=120,
            )
            respuesta.raise_for_status()
            contenido = respuesta.json()["message"]["content"]
            orden = json.loads(contenido)
            if not isinstance(orden, dict):
                raise ValueError("Se esperaba un objeto JSON")
            if orden.get("accion") == "navegar":
                destino = orden.get("destino")
                reconocido = texto.lower()
                if destino not in DESTINOS or destino not in reconocido:
                    orden = {
                        "accion": "responder",
                        "texto": "No entendí el destino. Di inicio o punto de prueba."
                    }
            self.historial.extend([
                {"role": "user", "content": texto},
                {"role": "assistant", "content": contenido},
            ])
            self.resultados.put((orden, None))
        except Exception as exc:
            self.resultados.put((None, str(exc)))

    def posicion(self):
        tf = self.tf_buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()
        )
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
        )

    def actualizar(self):
        try:
            orden, error = self.resultados.get_nowait()
        except queue.Empty:
            orden, error = None, None

        if error is not None:
            self.get_logger().error(error)
            self.hablar("No pude interpretar la solicitud. Inténtalo otra vez.")
            self.ocupado = False
        elif orden is not None:
            self.get_logger().info(
                "Interpretación: " + json.dumps(orden, ensure_ascii=False)
            )
            if orden.get("accion") == "navegar":
                self.preparar_ruta(orden.get("destino", ""))
            elif orden.get("accion") == "brazo":
                self.mover_brazo(orden.get("posicion", ""))
            elif orden.get("accion") == "reconocer":
                self.reconocer(orden.get("objeto", ""))
            elif orden.get("accion") == "responder":
                texto = orden.get("texto")
                if not isinstance(texto, str) or not texto.strip():
                    texto = "No pude interpretar la solicitud."
                self.hablar(texto[:500])
                self.ocupado = False
            else:
                self.hablar("Esa acción todavía no está disponible.")
                self.ocupado = False

        if self.meta is not None and self.llegada_recibida:
            try:
                x, y = self.posicion()
            except TransformException:
                return
            gx, gy = DESTINOS[self.meta]
            if math.hypot(x - gx, y - gy) <= 0.4:
                self.hablar("He llegado a " + self.meta + ".")
                self.meta = None
                self.llegada_recibida = False
                self.ocupado = False

    def preparar_ruta(self, destino):
        if not isinstance(destino, str) or destino not in DESTINOS:
            self.hablar("Ese destino no está configurado.")
            self.ocupado = False
            return
        if (
            not self.planificador.service_is_ready()
            or self.pub_meta.get_subscription_count() == 0
        ):
            self.hablar("La navegación no está disponible en este momento.")
            self.ocupado = False
            return
        try:
            x, y = self.posicion()
        except TransformException:
            self.hablar("Todavía no conozco mi posición. Inténtalo después.")
            self.ocupado = False
            return

        gx, gy = DESTINOS[destino]
        if math.hypot(x - gx, y - gy) <= 0.3:
            self.hablar("Ya estoy en " + destino + ".")
            self.ocupado = False
            return

        req = GetPlan.Request()
        req.start.header.frame_id = "map"
        req.start.pose.position.x = float(x)
        req.start.pose.position.y = float(y)
        req.start.pose.orientation.w = 1.0
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = gx
        req.goal.pose.position.y = gy
        req.goal.pose.orientation.w = 1.0
        futuro = self.planificador.call_async(req)
        futuro.add_done_callback(
            lambda f: self.enviar_meta(f, destino)
        )

    def enviar_meta(self, futuro, destino):
        try:
            respuesta = futuro.result()
            if respuesta is None or not respuesta.plan.poses:
                raise ValueError("El planificador no devolvió una ruta")
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.hablar("No encontré una ruta para llegar a ese destino.")
            self.ocupado = False
            return

        gx, gy = DESTINOS[destino]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self.meta = destino
        self.llegada_recibida = False
        self.pub_meta.publish(msg)
        self.hablar("Voy a " + destino + ".")

    def recibir_articulaciones(self, msg):
        nombres = [f"joint{i}" for i in range(1, 7)]
        if not all(nombre in msg.name for nombre in nombres):
            return
        if len(msg.position) != len(msg.name):
            return
        valores = [float(msg.position[msg.name.index(n)]) for n in nombres]
        if all(math.isfinite(v) for v in valores):
            self.articulaciones = valores
            self.hora_articulaciones = time.monotonic()

    def mover_brazo(self, posicion):
        posiciones = {"prueba": -0.5, "observacion": -1.2}
        if not isinstance(posicion, str) or posicion not in posiciones:
            self.hablar("Esa posición del brazo no está configurada.")
            self.ocupado = False
            return
        if (
            self.articulaciones is None
            or time.monotonic() - self.hora_articulaciones > 20
        ):
            self.hablar("No recibo la posición actual del brazo.")
            self.ocupado = False
            return
        if not self.cliente_brazo.server_is_ready():
            self.hablar("El controlador del brazo no está disponible.")
            self.ocupado = False
            return

        objetivo = self.articulaciones.copy()
        objetivo[4] = posiciones[posicion]
        if abs(self.articulaciones[4] - objetivo[4]) < 0.03:
            self.hablar("El brazo ya está en esa posición.")
            self.ocupado = False
            return

        solicitud = FollowJointTrajectory.Goal()
        solicitud.trajectory.joint_names = [
            f"joint{i}" for i in range(1, 7)
        ]
        punto = JointTrajectoryPoint()
        punto.positions = objetivo
        punto.time_from_start.sec = 4
        solicitud.trajectory.points = [punto]
        self.hablar("Voy a mover el brazo a posición de " + posicion + ".")
        futuro = self.cliente_brazo.send_goal_async(solicitud)
        futuro.add_done_callback(self.brazo_aceptado)

    def brazo_aceptado(self, futuro):
        try:
            manejador = futuro.result()
            if manejador is None or not manejador.accepted:
                raise RuntimeError("El controlador rechazó el movimiento.")
            resultado = manejador.get_result_async()
            resultado.add_done_callback(self.brazo_terminado)
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.hablar("No pude iniciar el movimiento del brazo.")
            self.ocupado = False

    def brazo_terminado(self, futuro):
        try:
            resultado = futuro.result()
            if (
                resultado.status == GoalStatus.STATUS_SUCCEEDED
                and resultado.result.error_code == 0
            ):
                self.hablar("Movimiento del brazo completado.")
            else:
                self.get_logger().error(
                    "Resultado del brazo: "
                    + str(resultado.result.error_code)
                    + " " + resultado.result.error_string
                )
                self.hablar("El brazo no pudo completar el movimiento.")
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.hablar("No pude confirmar el resultado del brazo.")
        finally:
            self.ocupado = False

    def reconocer(self, objeto):
        if not isinstance(objeto, str):
            self.hablar("No entendí qué objeto debo reconocer.")
            self.ocupado = False
            return
        if not self.vision.service_is_ready():
            self.hablar("El reconocimiento de objetos no está disponible.")
            self.ocupado = False
            return
        solicitud = RecognizeObject.Request()
        solicitud.id = objeto.strip().lower()
        self.hablar("Voy a revisar la imagen de la cámara.")
        futuro = self.vision.call_async(solicitud)
        futuro.add_done_callback(self.recibir_deteccion)

    def recibir_deteccion(self, futuro):
        traducciones = {
            "chair": "una silla",
            "couch": "un sofá",
            "sofa": "un sofá",
            "bottle": "una botella",
            "cup": "una taza",
            "person": "una persona",
            "book": "un libro",
            "dining table": "una mesa",
            "tv": "un televisor",
            "cell phone": "un teléfono",
        }
        try:
            respuesta = futuro.result()
            if respuesta is None:
                raise RuntimeError("El servicio no devolvió una respuesta.")
            objeto = respuesta.obj
            if objeto.id and objeto.confidence > 0.0:
                nombre = traducciones.get(objeto.id, objeto.id)
                self.hablar("Detecté " + nombre + ".")
                self.get_logger().info(
                    f"Detección: {objeto.id}; confianza: {objeto.confidence:.2f}"
                )
            else:
                self.hablar(
                    "No pude detectar el objeto en la imagen actual."
                )
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.hablar("No pude completar el reconocimiento.")
        finally:
            self.ocupado = False

    def recibir_llegada(self, msg):
        if msg.data and self.meta is not None:
            self.llegada_recibida = True


def main(args=None):
    rclpy.init(args=args)
    nodo = OllamaPlanningNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
