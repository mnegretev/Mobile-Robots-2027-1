"""
MOBILE ROBOTS - FI-UNAM, 2027-1
PROYECTO FINAL: Robot de servicio controlado por lenguaje natural.

Flujo:
  micrófono -> speech2text (Whisper) -> /sp_rec/recognized -> [este nodo]
  [este nodo] -> NLU -> ejecuta la tarea (navegar, reconocer, brazo, responder)
  [este nodo] -> /tts_query -> text2speech (Piper) -> bocina

Habilidades:
  * Navegar a un lugar conocido      ("ve a la cocina")
  * Reconocer objetos con la cámara  ("qué ves", "busca la mostaza")
  * Mover el manipulador             ("levanta el brazo", "saluda", "toma la lata")
  * Contestar preguntas              ("dónde estás", "qué hora es", preguntas generales con LLM)
  * Decir con voz cuando no puede hacer algo y por qué.

Para probar sin micrófono:
  ros2 topic pub --once /sp_rec/recognized std_msgs/msg/String "{data: 've a la cocina'}"
"""
import math
import os
import queue
import re
import threading
import time
from datetime import datetime

import numpy
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from nav_msgs.srv import GetPlan, GetMap
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

from final_project.nlu import NLU, normalize

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import requests
except ImportError:
    requests = None

# Frases que Whisper inventa cuando oye ruido (vienen de subtítulos de YouTube)
HALLUCINATIONS = ["suscribete", "amara org", "subtitulos", "proximo video", "gracias por ver",
                  "hasta la proxima", "dale like", "nos vemos"]

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

DAYS = ["lunes", "martes", "miércoles", "jueves", "viernes", "sábado", "domingo"]
MONTHS = ["enero", "febrero", "marzo", "abril", "mayo", "junio", "julio", "agosto",
          "septiembre", "octubre", "noviembre", "diciembre"]

# Rangos HSV de OpenCV (H en [0,180]) para el detector por color.
COLOR_RANGES = {
    "rojo":     [((0, 120, 70), (8, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "naranja":  [((9, 140, 100), (19, 255, 255))],
    "amarillo": [((20, 120, 100), (35, 255, 255))],
    "verde":    [((40, 80, 50), (85, 255, 255))],
    "azul":     [((95, 120, 50), (130, 255, 255))],
}

# Traducción de algunas clases COCO que puede devolver YOLO.
COCO_ES = {
    "bottle": "una botella", "cup": "una taza o vaso", "bowl": "un tazón", "chair": "una silla",
    "couch": "un sillón", "bed": "una cama", "dining table": "una mesa", "tv": "una televisión",
    "refrigerator": "un refrigerador", "potted plant": "una planta", "book": "un libro",
    "laptop": "una computadora", "cell phone": "un celular", "vase": "un florero",
    "sports ball": "una pelota", "clock": "un reloj", "person": "una persona",
    "banana": "un plátano", "apple": "una manzana", "orange": "una naranja",
}


class VoiceCommanderNode(Node):
    def __init__(self):
        super().__init__("voice_commander")
        self.get_logger().info("INITIALIZING VOICE COMMANDER NODE (PROYECTO FINAL)")
        default_kb = os.path.join(get_package_share_directory("final_project"), "config", "knowledge.yaml")
        self.declare_parameter("knowledge_file", default_kb)
        self.declare_parameter("use_llm", True)
        self.declare_parameter("llm_url", "http://localhost:11434/api/chat")
        self.declare_parameter("llm_model", "llama3")
        self.declare_parameter("use_yolo", True)
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("arm_topic", "/xarm6_traj_controller/joint_trajectory")
        self.declare_parameter("grasp_requires_detection", True)
        self.declare_parameter("require_name", False)       # True: solo obedece si lo llaman por su nombre
        self.declare_parameter("seconds_per_char", 0.075)   # para no escucharse a sí mismo mientras habla
        self.declare_parameter("nav_timeout", 40.0)

        kb_file = self.get_parameter("knowledge_file").value
        self.kb = yaml.safe_load(open(kb_file))
        self.robot_name = self.kb.get("robot_name", "Robot")
        self.locations = self.kb.get("locations", {})
        self.objects = self.kb.get("objects", {})
        self.arm_poses = self.kb.get("arm_poses", {})
        self.gripper_cfg = self.kb.get("gripper", {})
        self.nlu = NLU(self.kb, self.robot_name)

        self.tasks = queue.Queue()
        self.busy = False
        self.cancel = False
        self.goal_reached = False
        self.mute_until = 0.0
        self.tts_speaking = False
        self.tts_end_time = 0.0
        self.recent_speech = []        # (tiempo, palabras) de lo que dijo el robot
        self.last_unknown_reply = 0.0
        self.last_busy_reply = 0.0
        self.goal_failed = False
        self.last_image = None
        self.inflated_map = None
        self.yolo = None
        self.yolo_failed = False
        self.llm_history = []

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_tts = self.create_publisher(String, "/tts_query", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 1)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_nav_stop = self.create_publisher(Bool, "/navigation/stop", 1)
        self.pub_arm = self.create_publisher(JointTrajectory, self.get_parameter("arm_topic").value, 1)
        self.pub_gripper = self.create_publisher(JointTrajectory,
                                                 self.gripper_cfg.get("topic", "/xarm_gripper_traj_controller/joint_trajectory"), 1)
        self.create_subscription(String, "/sp_rec/recognized", self.callback_speech, 1, callback_group=cb)
        self.create_subscription(String, "/voice_commander/text", self.callback_text, 1, callback_group=cb)
        self.create_subscription(Bool, "/navigation/goal_reached", self.callback_goal_reached, 1, callback_group=cb)
        self.create_subscription(Image, self.get_parameter("camera_topic").value, self.callback_image, 1, callback_group=cb)
        self.create_subscription(Bool, "/tts_speaking", self.callback_tts_speaking, 10, callback_group=cb)
        self.clt_plan = self.create_client(GetPlan, "/path_planning/plan_path", callback_group=cb)
        self.clt_map = self.create_client(GetMap, "/get_inflated_map", callback_group=cb)

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

    # ================================================================
    # Callbacks
    # ================================================================
    def callback_tts_speaking(self, msg):
        self.tts_speaking = msg.data
        if not msg.data:
            self.tts_end_time = time.time()

    def robot_is_talking(self):
        if self.tts_speaking:
            return True
        if time.time() - self.tts_end_time < 1.5:
            return True
        if self.pub_tts.get_subscription_count() > 0 and time.time() < self.mute_until:
            return True   # respaldo por si text2speech no publica /tts_speaking
        return False

    def is_echo(self, text):
        """True si lo que se escuchó se parece a algo que el robot dijo hace poco."""
        words = [w for w in normalize(text).split() if len(w) > 2]
        if len(words) == 0:
            return False
        now = time.time()
        self.recent_speech = [(t, w) for t, w in self.recent_speech if now - t < 40.0]
        for t, said in self.recent_speech:
            common = 0
            for w in words:
                if w in said:
                    common += 1
            if common >= 0.6 * len(words):
                return True
        return False

    def is_hallucination(self, text):
        t = normalize(text)
        for h in HALLUCINATIONS:
            if h in t:
                return True
        return False

    def callback_speech(self, msg):
        text = msg.data
        if self.robot_is_talking():
            self.get_logger().info("Ignorando (el robot está hablando): " + text)
            return
        if self.is_hallucination(text):
            self.get_logger().info("Ignorando alucinación de Whisper: " + text)
            return
        if self.is_echo(text):
            self.get_logger().info("Ignorando eco de mi propia voz: " + text)
            return
        self.handle_text(text, from_voice=True)

    def callback_text(self, msg):  # órdenes escritas, sin filtros
        self.handle_text(msg.data, from_voice=False)

    def handle_text(self, text, from_voice=False):
        text = text.strip()
        if not text:
            return
        self.get_logger().info("Escuché: '" + text + "'")
        if self.get_parameter("require_name").value and self.robot_name.lower() not in text.lower():
            self.get_logger().info("No me llamaron por mi nombre, ignoro la frase.")
            return
        intents = self.nlu.parse(text)
        self.get_logger().info("Intenciones: " + str(intents))
        if not intents:
            return
        if intents[0]["type"] == "stop":
            self.stop_everything()
            return
        understood = False
        for i in intents:
            if i["type"] != "unknown":
                understood = True
        if from_voice and not understood:
            # Ruido o una frase sin sentido: se avisa como máximo una vez cada 20 s
            if self.busy or time.time() - self.last_unknown_reply < 20.0:
                self.get_logger().info("Frase no entendida, no respondo: " + text)
                return
            self.last_unknown_reply = time.time()
        if self.busy:
            if understood and time.time() - self.last_busy_reply > 10.0:
                self.last_busy_reply = time.time()
                self.say("Estoy ocupado con otra tarea. Di alto si quieres que me detenga.")
            return
        self.tasks.put(intents)

    def callback_goal_reached(self, msg):
        if msg.data:
            self.goal_reached = True
        else:
            self.goal_failed = True

    def callback_image(self, msg):
        self.last_image = msg

    # ================================================================
    # Utilidades
    # ================================================================
    def say(self, text):
        self.get_logger().info("Robot dice: " + text)
        self.pub_tts.publish(String(data=text))
        self.recent_speech.append((time.time(), set(w for w in normalize(text).split() if len(w) > 2)))
        speak_time = self.get_parameter("seconds_per_char").value * len(text) + 1.5
        self.mute_until = max(self.mute_until, time.time()) + speak_time

    def wait_speaking(self):
        while time.time() < self.mute_until - 1.0 and not self.cancel:
            time.sleep(0.1)

    def sleep(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds and not self.cancel and rclpy.ok():
            time.sleep(0.05)

    def wait_future(self, future, timeout):
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        if future.done():
            return future.result()
        return None

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            q = t.transform.rotation
            a = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, a
        except TransformException:
            return None

    def nearest_location(self):
        pose = self.get_robot_pose()
        if pose is None:
            return None, None
        best, best_d = None, float("inf")
        for key, loc in self.locations.items():
            d = math.hypot(loc["x"] - pose[0], loc["y"] - pose[1])
            if d < best_d:
                best, best_d = key, d
        return best, best_d

    def loc_name(self, key):
        return self.locations[key].get("say", key)

    def obj_name(self, key):
        return self.objects[key].get("say", key)

    def stop_everything(self):
        self.get_logger().warn("ORDEN DE ALTO")
        self.cancel = True
        self.pub_nav_stop.publish(Bool(data=True))   # pure_pursuit deja de seguir la ruta
        for _ in range(5):
            self.pub_cmd_vel.publish(Twist())
        self.say("Me detengo.")

    # ================================================================
    # Ciclo de ejecución de tareas
    # ================================================================
    def worker_loop(self):
        time.sleep(3.0)
        t0 = time.time()
        while self.pub_tts.get_subscription_count() == 0 and time.time() - t0 < 15:
            time.sleep(0.5)
        self.say(f"Hola, soy {self.robot_name}. Estoy listo para recibir órdenes.")
        while rclpy.ok():
            try:
                intents = self.tasks.get(timeout=0.5)
            except queue.Empty:
                continue
            self.busy = True
            self.cancel = False
            try:
                for i, intent in enumerate(intents):
                    ok = self.execute(intent)
                    if self.cancel:
                        break
                    if not ok and i < len(intents) - 1:
                        self.say("Por eso no haré el resto de la tarea.")
                        break
            except Exception as e:  # nunca dejar al robot mudo por una excepción
                self.get_logger().error("Error ejecutando tarea: " + repr(e))
                self.say("Tuve un error interno y no pude terminar la tarea.")
            self.busy = False

    def execute(self, intent):
        t = intent["type"]
        if t == "navigate":
            return self.navigate(intent["location"])
        if t == "navigate_unknown":
            places = ", ".join(self.loc_name(k) for k in list(self.locations)[:6])
            self.say(f"No puedo ir a {intent['name'] or 'ese lugar'} porque no lo conozco. Conozco {places}.")
            return False
        if t == "find":
            return self.find_object(intent["object"])
        if t == "find_unknown":
            self.say(f"No conozco el objeto {intent['name']}, así que no puedo reconocerlo.")
            return False
        if t == "describe_scene":
            return self.describe_scene()
        if t == "grasp":
            return self.grasp(intent["object"], intent.get("bring", False))
        if t == "grasp_unknown":
            self.say(f"No puedo tomar {intent['name'] or 'eso'} porque no conozco ese objeto.")
            return False
        if t == "arm":
            return self.arm_action(intent)
        if t == "arm_unknown":
            self.say("No conozco ese movimiento del brazo. Puedo levantarlo, bajarlo, estirarlo, saludar, abrir o cerrar la pinza.")
            return False
        if t == "question":
            return self.answer(intent)
        if t == "impossible":
            self.say(f"Lo siento, no puedo {intent['action']}. No tengo esa capacidad.")
            return False
        self.say("No entendí qué debo hacer. Puedo navegar, reconocer objetos, mover el brazo y contestar preguntas.")
        return False

    # ================================================================
    # Navegación
    # ================================================================
    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.pub_goal.publish(msg)

    def snap_to_free(self, x, y, radius=1.5):
        """Mueve la meta a la celda libre más cercana del mapa inflado (si está disponible)."""
        if self.inflated_map is None and self.clt_map.wait_for_service(timeout_sec=0.5):
            resp = self.wait_future(self.clt_map.call_async(GetMap.Request()), 5.0)
            if resp is not None:
                self.inflated_map = resp.map
        if self.inflated_map is None:
            return x, y
        info = self.inflated_map.info
        grid = numpy.asarray(self.inflated_map.data).reshape((info.height, info.width))
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        r, c = int((y - oy) / res), int((x - ox) / res)
        if 0 <= r < info.height and 0 <= c < info.width and grid[r, c] == 0:
            return x, y
        k = int(radius / res)
        r0, c0 = max(r - k, 0), max(c - k, 0)
        window = grid[r0:r + k + 1, c0:c + k + 1]
        free = numpy.argwhere(window == 0)
        if len(free) == 0:
            return None
        d = (free[:, 0] + r0 - r) ** 2 + (free[:, 1] + c0 - c) ** 2
        fr, fc = free[numpy.argmin(d)]
        return (fc + c0) * res + ox + res / 2, (fr + r0) * res + oy + res / 2

    def navigate(self, key, announce=True):
        loc = self.locations[key]
        name = self.loc_name(key)
        if self.pub_goal.get_subscription_count() == 0:
            self.say(f"No puedo ir a {name} porque el sistema de navegación no está activo.")
            return False
        pose = self.get_robot_pose()
        if pose is None:
            self.say("No puedo navegar porque no sé dónde estoy. La localización no está funcionando.")
            return False
        if math.hypot(loc["x"] - pose[0], loc["y"] - pose[1]) < 0.45:
            if announce:
                self.say(f"Ya estoy en {name}.")
            return True
        goal = self.snap_to_free(loc["x"], loc["y"])
        if goal is None:
            self.say(f"No puedo ir a {name} porque ese lugar está ocupado por obstáculos.")
            return False
        gx, gy = goal
        # Verificar que existe un camino antes de moverse
        if self.clt_plan.wait_for_service(timeout_sec=0.5):
            req = GetPlan.Request()
            req.start.pose.position.x, req.start.pose.position.y = float(pose[0]), float(pose[1])
            req.goal.pose.position.x, req.goal.pose.position.y = float(gx), float(gy)
            resp = self.wait_future(self.clt_plan.call_async(req), 10.0)
            if resp is not None and len(resp.plan.poses) == 0:
                self.say(f"No encuentro un camino para llegar a {name}.")
                return False
        if announce:
            self.say(f"Voy a {name}.")
        self.goal_reached = False
        self.goal_failed = False
        self.publish_goal(gx, gy, loc.get("yaw", 0.0))
        dist = math.hypot(gx - pose[0], gy - pose[1])
        timeout = self.get_parameter("nav_timeout").value + 8.0 * dist
        t0 = time.time()
        while not self.goal_reached and not self.goal_failed and not self.cancel and time.time() - t0 < timeout:
            time.sleep(0.1)
        if self.cancel:
            return False
        if not self.goal_reached:
            self.pub_nav_stop.publish(Bool(data=True))
            self.say(f"No pude llegar a {name}. Tal vez el camino está bloqueado.")
            return False
        if announce:
            self.say(f"Llegué a {name}.")
        return True

    # ================================================================
    # Visión
    # ================================================================
    def get_bgr_image(self):
        msg = self.last_image
        if msg is None:
            return None
        if msg.encoding in ("rgba8", "bgra8"):
            ch = 4
        else:
            ch = 3
        img = numpy.frombuffer(bytes(msg.data), dtype=numpy.uint8).reshape((msg.height, msg.step))
        img = img[:, :msg.width * ch].reshape((msg.height, msg.width, ch))[:, :, :3]
        if msg.encoding.startswith("rgb"):
            img = img[:, :, ::-1]
        return numpy.ascontiguousarray(img)

    def detect_colors(self, img):
        if cv2 is None:
            return []
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        min_area = 0.002 * img.shape[0] * img.shape[1]
        found = []
        for color, ranges in COLOR_RANGES.items():
            mask = numpy.zeros(img.shape[:2], numpy.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, numpy.array(lo), numpy.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, numpy.ones((5, 5), numpy.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            areas = [cv2.contourArea(c) for c in contours if cv2.contourArea(c) > min_area]
            if areas:
                found.append({"source": "color", "label": color, "area": max(areas)})
        return found

    def detect_yolo(self, img):
        if not self.get_parameter("use_yolo").value or self.yolo_failed:
            return []
        try:
            if self.yolo is None:
                from ultralytics import YOLO
                self.yolo = YOLO(self.get_parameter("yolo_model").value)
            result = self.yolo(img, verbose=False, device="cpu", conf=0.35)[0]
            return [{"source": "yolo", "label": result.names[int(b.cls)], "conf": float(b.conf)}
                    for b in result.boxes]
        except Exception as e:
            self.get_logger().warn("YOLO no disponible, uso solo detección por color: " + repr(e))
            self.yolo_failed = True
            return []

    def look_and_detect(self):
        """Pone el brazo (con la cámara) en posición de observar y detecta objetos."""
        if self.pub_arm.get_subscription_count() > 0:
            self.move_arm("mirar")
        self.sleep(1.0)
        img = self.get_bgr_image()
        if img is None:
            return None
        return self.detect_colors(img) + self.detect_yolo(img)

    def matches(self, obj_key, detections):
        obj = self.objects[obj_key]
        for d in detections:
            if d["source"] == "yolo" and d["label"] in obj.get("yolo", []):
                return True
            if d["source"] == "color" and d["label"] in obj.get("colors", []):
                return True
        return False

    def describe_scene(self):
        dets = self.look_and_detect()
        if dets is None:
            self.say("No puedo ver nada porque no recibo imagen de la cámara.")
            return False
        here, dist = self.nearest_location()
        here_objs = []
        if dist is not None and dist < 1.5:
            here_objs = [k for k, o in self.objects.items() if o.get("location") == here]
        phrases = []
        for d in dets:
            if d["source"] == "yolo":
                p = COCO_ES.get(d["label"], "un objeto tipo " + d["label"])
            else:
                guess = [k for k in here_objs if d["label"] in self.objects[k].get("colors", [])]
                p = f"algo {d['label']}"
                if guess:
                    p = p + f", probablemente {self.obj_name(guess[0])}"
            if p not in phrases:
                phrases.append(p)
        if not phrases:
            self.say("No reconozco ningún objeto frente a mí.")
        else:
            self.say("Veo " + ", ".join(phrases[:5]) + ".")
        return True

    def find_object(self, key):
        name = self.obj_name(key)
        loc = self.objects[key].get("location")
        here, dist = self.nearest_location()
        if loc in self.locations and (here != loc or dist is None or dist > 1.0):
            self.say(f"{name.capitalize()} normalmente está en {self.loc_name(loc)}. Voy a buscar ahí.")
            if not self.navigate(loc, announce=False):
                return False
        dets = self.look_and_detect()
        if dets is None:
            self.say(f"No puedo buscar {name} porque no recibo imagen de la cámara.")
            return False
        if self.matches(key, dets):
            self.say(f"Encontré {name}. Está frente a mí.")
            return True
        self.say(f"Busqué {name} pero no la encuentro aquí.")
        return False

    # ================================================================
    # Manipulador
    # ================================================================
    def move_arm(self, pose, duration=2.0):
        if isinstance(pose, str):
            q = self.arm_poses[pose]
        else:
            q = pose
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = int(duration)
        p.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(p)
        self.pub_arm.publish(msg)
        self.sleep(duration + 0.4)

    def gripper(self, close):
        if self.pub_gripper.get_subscription_count() == 0:
            return False
        msg = JointTrajectory()
        msg.joint_names = [self.gripper_cfg.get("joint", "drive_joint")]
        p = JointTrajectoryPoint()
        if close:
            p.positions = [float(self.gripper_cfg.get("closed", 0.85))]
        else:
            p.positions = [float(self.gripper_cfg.get("open", 0.0))]
        p.time_from_start.sec = 1
        msg.points.append(p)
        self.pub_gripper.publish(msg)
        self.sleep(1.3)
        return True

    def arm_available(self):
        if self.pub_arm.get_subscription_count() == 0:
            self.say("No puedo mover el brazo porque su controlador no está activo.")
            return False
        return True

    def arm_action(self, intent):
        a = intent["action"]
        if a in ("gripper_open", "gripper_close"):
            if not self.gripper(a == "gripper_close"):
                self.say("No puedo usar la pinza porque no tengo un controlador de pinza disponible.")
                return False
            if a == "gripper_open":
                self.say("Listo, abrí la pinza.")
            else:
                self.say("Listo, cerré la pinza.")
            return True
        if not self.arm_available():
            return False
        if a == "wave":
            self.say("¡Hola!")
            for p in ["saludo_a", "saludo_b", "saludo_a", "saludo_b", "home"]:
                self.move_arm(p, 1.2)
            return True
        pose = intent["pose"]
        if pose not in self.arm_poses:
            self.say("No conozco esa posición del brazo.")
            return False
        texts = {"levantar": "Levanto el brazo.", "estirar": "Estiro el brazo.",
                 "home": "Pongo el brazo en reposo.", "mirar": "Apunto la cámara hacia el frente."}
        self.say(texts.get(pose, "Muevo el brazo."))
        self.move_arm(pose)
        return True

    def grasp(self, key, bring):
        name = self.obj_name(key)
        if not self.arm_available():
            return False
        loc = self.objects[key].get("location")
        here, dist = self.nearest_location()
        if loc in self.locations and (here != loc or dist is None or dist > 1.0):
            self.say(f"Voy por {name} a {self.loc_name(loc)}.")
            if not self.navigate(loc, announce=False):
                return False
        dets = self.look_and_detect()
        if self.get_parameter("grasp_requires_detection").value:
            if dets is None:
                self.say(f"No puedo tomar {name} porque no recibo imagen de la cámara.")
                self.move_arm("home")
                return False
            if not self.matches(key, dets):
                self.say(f"No veo {name} aquí, así que no puedo tomar ese objeto.")
                self.move_arm("home")
                return False
        self.say(f"Veo {name}. Voy a tomar ese objeto.")
        self.gripper(close=False)
        for p in ["pre_tomar", "tomar"]:
            self.move_arm(p)
        self.gripper(close=True)
        self.move_arm("cargar")
        if self.cancel:
            return False
        self.say(f"Tomé {name}.")
        if bring:
            if not self.navigate("inicio", announce=False):
                return False
            self.move_arm("estirar")
            self.say(f"Aquí tienes {name}.")
            self.gripper(close=False)
            self.move_arm("home")
        return True

    # ================================================================
    # Preguntas
    # ================================================================
    def answer(self, intent):
        k = intent["kind"]
        if k == "name":
            self.say(f"Me llamo {self.robot_name}. Soy un robot de servicio de la clase de robots móviles.")
        elif k == "abilities":
            self.say("Puedo ir a lugares de la casa, reconocer objetos con mi cámara, "
                     "mover mi brazo, tomar objetos y contestar preguntas.")
        elif k == "how_are_you":
            self.say("Estoy bien, con la batería llena y listo para ayudarte.")
        elif k == "where_am_i":
            here, dist = self.nearest_location()
            if here is None:
                self.say("No sé dónde estoy, la localización no está disponible.")
                return False
            if dist < 1.5:
                self.say(f"Estoy en {self.loc_name(here)}.")
            else:
                x, y, _ = self.get_robot_pose()
                self.say(f"Estoy en x {x:.1f}, y {y:.1f} metros, cerca de {self.loc_name(here)}.")
        elif k == "time":
            now = datetime.now()
            self.say(f"Son las {now.hour} con {now.minute} minutos.")
        elif k == "date":
            now = datetime.now()
            self.say(f"Hoy es {DAYS[now.weekday()]} {now.day} de {MONTHS[now.month - 1]} de {now.year}.")
        elif k == "known_places":
            self.say("Conozco " + ", ".join(self.loc_name(x) for x in self.locations) + ".")
        elif k == "known_objects":
            self.say("Conozco " + ", ".join(self.obj_name(x) for x in self.objects) + ".")
        elif k == "object_location":
            o = intent["object"]
            self.say(f"{self.obj_name(o).capitalize()} normalmente está en {self.loc_name(self.objects[o]['location'])}.")
        elif k == "place_info":
            self.say(f"Sí conozco {self.loc_name(intent['location'])}. Puedo llevarte si me lo pides.")
        elif k == "object_location_unknown":
            self.say(f"No sé dónde está {intent.get('name') or 'eso'}, no conozco ese objeto.")
            return False
        else:
            return self.answer_with_llm(intent.get("text", ""))
        return True

    def answer_with_llm(self, question):
        if not self.get_parameter("use_llm").value or requests is None:
            self.say("No sé la respuesta a esa pregunta.")
            return False
        if not self.llm_history:
            self.llm_history = [{"role": "system", "content":
                                 f"Eres {self.robot_name}, un robot de servicio en una casa simulada. "
                                 "Responde siempre en español, en una sola oración de máximo veinte palabras, "
                                 "sin emojis ni formato. Si no sabes algo, dilo."}]
        self.llm_history.append({"role": "user", "content": question})
        try:
            r = requests.post(self.get_parameter("llm_url").value,
                              json={"model": self.get_parameter("llm_model").value, "messages": self.llm_history[-9:],
                                    "stream": False, "options": {"num_ctx": 4096}}, timeout=40)
            text = r.json()["message"]["content"].strip()
            self.llm_history.append({"role": "assistant", "content": text})
        except Exception as e:
            self.get_logger().warn("LLM no disponible: " + repr(e))
            self.llm_history.pop()
            self.say("No sé la respuesta a esa pregunta.")
            return False
        text = re.sub(r"[*#_`]", "", text)
        words = text.split()
        self.say(" ".join(words[:40]))
        return True


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommanderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
