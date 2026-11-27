"""Execute spoken commands on the simulated service robot.

Listens to speech recognition, runs a skill when the robot can do it,
and says out loud when the order is impossible or fails.
"""

import json
import math
import socket
import threading
import time
import traceback
import urllib.error
import urllib.request
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from manip_msgs.srv import InverseKinematicsPose2Pose
from pumas_vision_msgs.srv import RecognizeObject, RecognizeObjects
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from final_project.interpreter import (
    ARM_POSES,
    JOINT_LIMITS,
    PLACES,
    interpret,
)


_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class VoiceSkillNode(Node):
    """Connect natural language with navigation, vision and the arm."""

    def __init__(self):
        super().__init__('voice_skill_node')
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434/api/chat')
        self.declare_parameter('ollama_model', 'llama3.2:1b')
        self.declare_parameter('use_llm', True)
        self.lock = threading.Lock()
        self.llm_lock = threading.Lock()
        self.active_model = None
        self.busy = False
        self.ignore_until = 0.0
        self.last_said = ''
        self.joints = [0.0] * 6
        self.goal_event = threading.Event()
        self.nav_ignore_until = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.speech_sub = self.create_subscription(
            String, '/sp_rec/recognized', self._on_text, 10)
        self.text_sub = self.create_subscription(
            String, '/hri/text_command', self._on_text, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)
        self.reached_sub = self.create_subscription(
            Bool, '/navigation/goal_reached', self._on_reached, 10)
        self.tts_pub = self.create_publisher(String, '/tts_query', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/xarm6_traj_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(String, '/hri/status', 10)
        self.plan_client = self.create_client(GetPlan, '/path_planning/plan_path')
        self.ik_client = self.create_client(
            InverseKinematicsPose2Pose, '/manipulation/ik_pose2pose')
        self.recog_one = self.create_client(
            RecognizeObject, '/vision/recognize_object')
        self.recog_all = self.create_client(
            RecognizeObjects, '/vision/recognize_objects')
        self.hello_timer = self.create_timer(6.0, self._hello_once)
        threading.Thread(target=self._warm_llm, daemon=True).start()
        self.get_logger().info(
            'Escuchando /sp_rec/recognized y /hri/text_command')

    def _hello_once(self):
        self.hello_timer.cancel()
        self._speak('Listo. Te escucho.')

    def _on_joints(self, msg):
        found = dict(zip(msg.name, msg.position))
        if not all(name in found for name in _JOINTS):
            return
        with self.lock:
            self.joints = [found[name] for name in _JOINTS]

    def _on_reached(self, msg):
        if msg.data and time.monotonic() >= self.nav_ignore_until:
            self.goal_event.set()

    def _on_text(self, msg):
        text = msg.data.strip()
        if len(text) < 2 or self._is_echo(text):
            return
        with self.lock:
            if self.busy or time.monotonic() < self.ignore_until:
                self.get_logger().info('Ignoro mientras estoy ocupado: ' + text)
                return
            self.busy = True
        worker = threading.Thread(target=self._run, args=(text,), daemon=True)
        worker.start()

    def _run(self, text):
        try:
            self._handle(text)
        except Exception as exc:
            self.get_logger().error(
                'No pude ejecutar la orden: %s\n%s' % (exc, traceback.format_exc()))
            detail = str(exc).strip().replace('\n', ' ')
            if len(detail) > 140:
                detail = detail[:140]
            if detail:
                self._speak('No pude completar la orden. ' + detail)
            else:
                self._speak('No pude completar la orden.')
        finally:
            with self.lock:
                self.busy = False

    def _handle(self, text):
        command = interpret(text)
        self.get_logger().info('Orden: %s -> %s' % (text, command.get('skill')))
        if command.get('try_llm') and self.get_parameter('use_llm').value:
            if command.get('skill') in ('answer', 'reject') and not command.get('dynamic'):
                spoken = self._chat_llm(text)
                command = {
                    'skill': 'answer',
                    'say': spoken or 'No pude consultar el modelo de lenguaje.',
                    'dynamic': None,
                    'try_llm': False,
                }
            else:
                refined = self._ask_llm(text)
                if refined is not None and self._llm_can_replace(command, refined):
                    command = refined
                    self.get_logger().info(
                        'El modelo eligió %s' % command.get('skill'))
        self._execute(command)

    def _execute(self, command):
        skill = command.get('skill')
        if skill == 'stop':
            self._speak(command['say'])
            self._stop_base()
            return
        if skill == 'answer':
            self._speak(self._answer_text(command))
            return
        if skill == 'reject':
            self._speak(command.get('say') or 'No puedo hacer eso.')
            return
        if skill == 'navigate':
            arrived = self._navigate(command)
            if arrived and command.get('then') == 'recognize':
                self._move_named_pose('mirar', speak=False)
                self._recognize(command.get('object'))
            return
        if skill == 'recognize':
            if command.get('move_arm'):
                self._speak(command.get('say') or 'Voy a mirar.')
                self._move_named_pose('mirar', speak=False)
            self._recognize(command.get('object'))
            return
        if skill == 'manipulate':
            self._manipulate(command)
            return
        if skill == 'spin':
            self._spin_base(command)
            return
        self._speak('No puedo hacer eso.')

    def _answer_text(self, command):
        kind = command.get('dynamic')
        if kind == 'pose':
            return self._where_am_i()
        if kind == 'time':
            now = datetime.now()
            return 'Son las %d y %d.' % (now.hour, now.minute)
        if kind == 'date':
            months = (
                'enero', 'febrero', 'marzo', 'abril', 'mayo', 'junio',
                'julio', 'agosto', 'septiembre', 'octubre', 'noviembre',
                'diciembre',
            )
            now = datetime.now()
            return 'Hoy es %d de %s.' % (now.day, months[now.month - 1])
        return command.get('say') or 'No tengo esa información.'

    def _speak(self, text):
        if not text:
            return
        self.get_logger().info('Digo: ' + text)
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        status = String()
        status.data = text
        self.status_pub.publish(status)
        self.last_said = text
        self.ignore_until = time.monotonic() + min(12.0, 0.08 * len(text) + 0.8)

    def _is_echo(self, text):
        heard = ' '.join(text.lower().split())
        said = ' '.join(self.last_said.lower().split())
        if len(heard) < 12 or len(said) < 12:
            return False
        return heard in said or said in heard

    def _where_am_i(self):
        pose = self._robot_pose()
        if pose is None:
            return 'No puedo leer mi posición en el mapa.'
        x_pos, y_pos, _yaw = pose
        nearest = None
        best = 1.6
        for spec in PLACES.values():
            dist = math.hypot(spec['x'] - x_pos, spec['y'] - y_pos)
            if dist < best:
                best = dist
                nearest = spec['label']
        if nearest:
            return 'Estoy cerca de %s.' % nearest
        return 'Estoy en el mapa, en %.1f, %.1f.' % (x_pos, y_pos)

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
        except TransformException:
            return None
        trans = transform.transform.translation
        rot = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rot.w * rot.z + rot.x * rot.y),
            1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))
        return trans.x, trans.y, yaw

    def _navigate(self, command):
        x_goal = float(command['x'])
        y_goal = float(command['y'])
        yaw = float(command.get('yaw') or 0.0)
        label = command.get('label') or 'ese lugar'
        if abs(x_goal) > 12.0 or abs(y_goal) > 10.0:
            self._speak('Ese punto está fuera de la casa. No puedo ir.')
            return False
        pose = self._robot_pose()
        if pose is None:
            self._speak('No sé dónde estoy. No puedo navegar.')
            return False
        if math.hypot(x_goal - pose[0], y_goal - pose[1]) < 0.45:
            self._speak('Ya estoy en %s.' % label)
            return True
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self._speak('No puedo navegar. El planificador no está listo.')
            return False
        request = GetPlan.Request()
        request.start.pose.position.x = float(pose[0])
        request.start.pose.position.y = float(pose[1])
        request.goal.pose.position.x = x_goal
        request.goal.pose.position.y = y_goal
        future = self.plan_client.call_async(request)
        if not self._wait_future(future, 10.0):
            self._speak('No pude calcular un camino. No voy a moverme.')
            return False
        plan = future.result().plan
        if plan is None or len(plan.poses) < 2:
            self._speak(
                'No hay camino libre hacia %s. No puedo llegar.' % label)
            return False
        self._speak(command.get('say') or 'Voy para allá.')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_event.clear()
        self.nav_ignore_until = time.monotonic() + 0.5
        self.goal_pub.publish(goal)
        timeout = min(180.0, 20.0 + 8.0 * math.hypot(
            x_goal - pose[0], y_goal - pose[1]))
        if not self.goal_event.wait(timeout):
            self._stop_base()
            self._speak('No alcancé %s. Me detengo.' % label)
            return False
        self._speak('Ya llegué a %s.' % label)
        return True

    def _spin_base(self, command):
        direction = 1.0 if float(command.get('direction', 1.0)) >= 0.0 else -1.0
        radians = abs(float(command.get('radians', 1.5708)))
        speed = 0.6
        sim_seconds = max(0.4, radians / speed)
        self._speak(command.get('say') or 'Estoy girando.')
        twist = Twist()
        twist.angular.z = direction * speed
        started = time.monotonic()
        goal = self.get_clock().now() + Duration(seconds=sim_seconds)
        while rclpy.ok() and time.monotonic() - started < 45.0:
            if self.get_clock().now() >= goal:
                break
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self._stop_base()

    def _stop_base(self):
        msg = Twist()
        for _index in range(8):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def _manipulate(self, command):
        mode = command.get('mode', 'named')
        if mode == 'named':
            self._move_named_pose(command.get('pose') or 'navegacion', speak=True)
            return
        if mode == 'relative':
            self._move_relative(command)
            return
        if mode == 'joints':
            joints = self._clamp(command.get('joints') or [])
            if joints is None:
                self._speak('Esos ángulos están fuera del alcance del brazo.')
                return
            self._speak(command.get('say') or 'Muevo el brazo.')
            self._publish_joints(joints)
            return
        if mode == 'cartesian':
            self._move_cartesian(command)
            return
        self._speak('No puedo mover el brazo de esa forma.')

    def _move_named_pose(self, pose_name, speak):
        spec = ARM_POSES.get(pose_name)
        if spec is None:
            self._speak('No conozco esa posición del brazo.')
            return False
        if speak:
            self._speak('Muevo el brazo a la posición %s.' % spec['label'])
        self._publish_joints(spec['joints'])
        return True

    def _move_relative(self, command):
        index = int(command.get('joint', 0))
        delta = float(command.get('delta', 0.0))
        with self.lock:
            joints = list(self.joints)
        joints[index] = joints[index] + delta
        limited = self._clamp(joints)
        if limited is None:
            self._speak('No puedo mover más el brazo en esa dirección.')
            return
        self._speak(command.get('say') or 'Muevo el brazo.')
        self._publish_joints(limited)

    def _move_cartesian(self, command):
        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self._speak('No puedo mover el brazo. La cinemática no está lista.')
            return
        with self.lock:
            guess = list(self.joints)
        request = InverseKinematicsPose2Pose.Request()
        request.x = float(command.get('x', 0.3))
        request.y = float(command.get('y', 0.0))
        request.z = float(command.get('z', 0.4))
        request.roll = float(command.get('roll', 3.14))
        request.pitch = float(command.get('pitch', 0.0))
        request.yaw = float(command.get('yaw_arm', 0.0))
        request.initial_guess = guess
        future = self.ik_client.call_async(request)
        if not self._wait_future(future, 8.0):
            self._speak('No pude calcular la cinemática inversa.')
            return
        solution = list(future.result().q)
        if len(solution) < 6:
            self._speak('Esa pose está fuera del alcance del brazo. No la muevo.')
            return
        clamped = self._clamp(solution[:6])
        if clamped is None:
            self._speak('La solución del brazo queda fuera de los límites.')
            return
        self._speak(command.get('say') or 'Muevo el brazo a esa pose.')
        self._publish_joints(clamped)

    def _clamp(self, joints):
        if len(joints) < 6:
            return None
        limited = []
        for value, (low, high) in zip(joints, JOINT_LIMITS):
            if value < low - 0.05 or value > high + 0.05:
                return None
            limited.append(min(high, max(low, float(value))))
        return limited

    def _publish_joints(self, joints):
        duration = 2.5
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in joints]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        msg.points.append(point)
        self.arm_pub.publish(msg)
        self._sleep_sim(duration + 0.8)

    def _recognize(self, object_name):
        if object_name:
            client = self.recog_one
            request = RecognizeObject.Request()
            request.id = object_name
        else:
            client = self.recog_all
            request = RecognizeObjects.Request()
        if not client.wait_for_service(timeout_sec=2.0):
            self._speak(
                'No puedo reconocer objetos. El nodo de visión no está activo.')
            return
        future = client.call_async(request)
        if not self._wait_future(future, 25.0):
            self._speak('No pude terminar el reconocimiento.')
            return
        result = future.result()
        if object_name:
            found = []
            if result.obj.id:
                found.append(result.obj.id)
            self._speak_recognition(object_name, found, result.obj)
            return
        names = [item.id for item in result.objs if item.id]
        if names == ['__sin_imagen__']:
            self._speak('No estoy recibiendo imagen de la cámara.')
            return
        self._speak_recognition(None, names, None)

    def _speak_recognition(self, requested, names, one):
        if requested and requested in names:
            conf = ''
            if one is not None and one.confidence > 0.0:
                conf = ', con confianza %.0f por ciento' % (100.0 * one.confidence)
            self._speak('Sí veo %s%s.' % (requested, conf))
            return
        if requested and not names:
            self._speak('No veo %s desde aquí. No puedo confirmarlo.' % requested)
            return
        if requested:
            self._speak(
                'No veo %s. Sí veo %s.' % (requested, ', '.join(names)))
            return
        if not names:
            self._speak('No veo ningún objeto conocido frente a la cámara.')
            return
        self._speak('Veo %s.' % ', '.join(names))

    def _sleep_sim(self, seconds):
        deadline = self.get_clock().now() + Duration(seconds=float(seconds))
        while rclpy.ok() and self.get_clock().now() < deadline:
            time.sleep(0.05)

    def _llm_can_replace(self, original, refined):
        if original.get('skill') != 'answer':
            return True
        return refined.get('skill') == 'answer'

    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result() is not None
                except Exception as exc:
                    self.get_logger().error('El servicio falló: %s' % exc)
                    return False
            time.sleep(0.05)
        return False

    def _chat_llm(self, text):
        """Ask the language model for a spoken answer, not a robot command."""
        prompt = (
            'Responde en español, en una sola frase corta, para decirla en voz alta. '
            'Eres un robot de servicio. Pregunta: ' + text
        )
        try:
            content = self._generate(prompt, 32, 100.0)
        except Exception as exc:
            self.get_logger().error('Sin modelo de lenguaje: %s' % exc)
            return self._llm_failure_say(exc)
        reply = self._plain_reply(content)
        if not reply:
            return ''
        self.get_logger().info('El modelo respondió: ' + reply)
        return reply

    def _llm_base(self):
        url = self.get_parameter('ollama_url').value or 'http://127.0.0.1:11434'
        if '://' not in url:
            url = 'http://' + url
        url = url.replace('://localhost', '://127.0.0.1')
        base = url.rstrip('/')
        for suffix in ('/api/chat', '/api/generate', '/api/tags'):
            if base.endswith(suffix):
                base = base[:-len(suffix)]
                break
        return base

    def _installed_models(self):
        request = urllib.request.Request(self._llm_base() + '/api/tags')
        with urllib.request.urlopen(request, timeout=4) as response:
            payload = json.loads(response.read().decode('utf-8'))
        names = []
        for item in payload.get('models') or []:
            name = item.get('name') or item.get('model')
            if name:
                names.append(name)
        return names

    def _choose_model(self, names):
        wanted = str(self.get_parameter('ollama_model').value or 'llama3.2:1b')
        if wanted in names:
            return wanted
        for name in (
            'llama3.2:1b', 'llama3.2:latest', 'tinyllama:latest',
            'tinyllama', 'phi3:mini', 'gemma2:2b',
        ):
            if name in names:
                return name
        small = [
            name for name in names
            if name not in ('llama3', 'llama3:latest') and ':8b' not in name
            and '70b' not in name
        ]
        if small:
            return small[0]
        if names:
            return names[0]
        return wanted

    def _model_name(self):
        if self.active_model:
            return self.active_model
        try:
            names = self._installed_models()
        except Exception as exc:
            raise RuntimeError('Ollama no responde: %s' % exc) from exc
        if not names:
            raise RuntimeError(
                'Ollama no tiene modelos. Ejecuta: ollama pull llama3.2:1b')
        chosen = self._choose_model(names)
        self.active_model = chosen
        self.get_logger().info(
            'Modelo de lenguaje: %s. Instalados: %s' % (chosen, ', '.join(names)))
        if chosen in ('llama3', 'llama3:latest') or ':8b' in chosen or '70b' in chosen:
            self.get_logger().error(
                'Ese modelo no alcanza a responder con Gazebo abierto. '
                'Ejecuta: ollama pull llama3.2:1b')
        return chosen

    def _running_models(self):
        request = urllib.request.Request(self._llm_base() + '/api/ps')
        with urllib.request.urlopen(request, timeout=4) as response:
            payload = json.loads(response.read().decode('utf-8'))
        names = []
        for item in payload.get('models') or []:
            name = item.get('name') or item.get('model')
            if name:
                names.append(name)
        return names

    def _unload_heavy_models(self, keep):
        try:
            names = self._running_models()
        except Exception:
            return
        for name in names:
            heavy = (
                name in ('llama3', 'llama3:latest')
                or ':8b' in name or '70b' in name or ':70b' in name
            )
            if not heavy or name == keep:
                continue
            body = json.dumps({
                'model': name,
                'keep_alive': 0,
                'prompt': '',
            }).encode('utf-8')
            request = urllib.request.Request(
                self._llm_base() + '/api/generate',
                data=body,
                headers={'Content-Type': 'application/json'},
            )
            try:
                with urllib.request.urlopen(request, timeout=8) as response:
                    response.read()
                self.get_logger().info('Saqué de memoria el modelo %s' % name)
            except Exception as exc:
                self.get_logger().info(
                    'No pude sacar de memoria %s: %s' % (name, exc))

    def _warm_llm(self):
        self.get_logger().info('Preparando el modelo de lenguaje')
        try:
            with self.llm_lock:
                model = self._model_name()
                self._unload_heavy_models(model)
                self._generate_with_model(model, 'Di hola.', 8, 120.0)
            self.get_logger().info('Modelo de lenguaje listo: %s' % model)
        except Exception as exc:
            self.get_logger().error('No pude preparar el modelo: %s' % exc)

    def _generate(self, prompt, num_predict, timeout):
        with self.llm_lock:
            model = self._model_name()
            return self._generate_with_model(model, prompt, num_predict, timeout)

    def _generate_with_model(self, model, prompt, num_predict, timeout):
        self.get_logger().info('Consultando %s' % model)
        body = json.dumps({
            'model': model,
            'prompt': prompt,
            'stream': True,
            'keep_alive': '30m',
            'options': {
                'num_ctx': 256,
                'num_predict': int(num_predict),
                'temperature': 0.2,
                'num_gpu': 0,
                'num_thread': 2,
            },
        }).encode('utf-8')
        request = urllib.request.Request(
            self._llm_base() + '/api/generate',
            data=body,
            headers={'Content-Type': 'application/json'},
        )
        deadline = time.monotonic() + timeout
        pieces = []
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                while time.monotonic() < deadline:
                    line = response.readline()
                    if not line:
                        break
                    try:
                        chunk = json.loads(line.decode('utf-8'))
                    except json.JSONDecodeError:
                        continue
                    if chunk.get('error'):
                        raise RuntimeError(str(chunk['error']))
                    pieces.append(str(chunk.get('response') or ''))
                    text = ''.join(pieces).strip()
                    if chunk.get('done') or self._reply_ready(text):
                        return text
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', 'replace')[:180]
            raise RuntimeError('HTTP %s %s' % (exc.code, detail)) from exc
        except socket.timeout as exc:
            text = ''.join(pieces).strip()
            if text:
                return text
            raise TimeoutError(
                'el modelo no alcanzó a responder a tiempo') from exc
        text = ''.join(pieces).strip()
        if text:
            return text
        raise TimeoutError('el modelo no alcanzó a responder a tiempo')

    def _reply_ready(self, text):
        if len(text) >= 24 and text[-1] in '.!?':
            return True
        return len(text) >= 160

    def _plain_reply(self, content):
        parsed = None
        try:
            parsed = self._parse_llm(content)
        except Exception:
            parsed = None
        if parsed and parsed.get('say'):
            reply = str(parsed['say']).strip()
        else:
            reply = str(content or '').strip().strip('`').strip()
        if len(reply) > 220:
            reply = reply[:220].rsplit(' ', 1)[0]
        return reply

    def _llm_failure_say(self, exc):
        message = str(exc).lower()
        if 'no tiene modelos' in message or '404' in message or 'not found' in message:
            return (
                'El modelo no está instalado. '
                'Hay que ejecutar ollama pull llama3.2 uno b.'
            )
        if (
            'connection refused' in message
            or 'urlopen error' in message
            or 'no responde' in message
        ):
            return 'Ollama no está corriendo, no puedo consultar el modelo.'
        return ''

    def _ask_llm(self, text):
        prompt = (
            'Responde solo JSON con skill, place, object, pose y say. '
            'skill es navigate, recognize, manipulate, answer o reject. '
            'Orden: ' + text
        )
        try:
            content = self._generate(prompt, 48, 40.0)
        except Exception as exc:
            self.get_logger().info('Sin modelo de lenguaje: %s' % exc)
            return None
        try:
            return self._parse_llm(content)
        except Exception as exc:
            self.get_logger().info('Respuesta del modelo no usable: %s' % exc)
            return None

    def _parse_llm(self, content):
        start = content.find('{')
        end = content.rfind('}')
        if start < 0 or end <= start:
            return None
        try:
            data = json.loads(content[start:end + 1])
        except json.JSONDecodeError:
            return None
        if not isinstance(data, dict):
            return None
        skill = str(data.get('skill', 'reject')).strip().lower()
        say = str(data.get('say') or '').strip()
        if skill == 'navigate':
            place = str(data.get('place') or '')
            if place not in PLACES:
                return {
                    'skill': 'reject',
                    'say': say or 'No conozco ese lugar y no puedo ir.',
                    'try_llm': False,
                }
            spec = PLACES[place]
            return {
                'skill': 'navigate',
                'place': place,
                'x': spec['x'],
                'y': spec['y'],
                'yaw': spec['yaw'],
                'label': spec['label'],
                'say': say or ('Voy hacia %s.' % spec['label']),
                'then': None,
                'object': None,
                'try_llm': False,
            }
        if skill == 'recognize':
            obj = data.get('object') or None
            return {
                'skill': 'recognize',
                'object': obj,
                'pose': 'mirar',
                'say': say or 'Voy a mirar.',
                'try_llm': False,
            }
        if skill == 'manipulate':
            pose = str(data.get('pose') or 'navegacion')
            if pose not in ARM_POSES:
                pose = 'navegacion'
            return {
                'skill': 'manipulate',
                'mode': 'named',
                'pose': pose,
                'say': say,
                'try_llm': False,
            }
        if skill == 'answer':
            return {
                'skill': 'answer',
                'say': say or 'No tengo esa información.',
                'dynamic': None,
                'try_llm': False,
            }
        return {
            'skill': 'reject',
            'say': say or 'No puedo hacer eso.',
            'try_llm': False,
        }


def main(args=None):
    rclpy.init(args=args)
    node = VoiceSkillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
