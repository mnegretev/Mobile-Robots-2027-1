#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# NLP & QA Orchestrator Node using Local Ollama 3
#

import json
import math
import os
import re
import time
import requests
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.srv import GetPlan
from pumas_vision_msgs.srv import RecognizeObject, RecognizeObjects
from manip_msgs.srv import InverseKinematicsPose2Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"

LOCATIONS = {
    "dinette": (6.59, -1.04),
    "kitchen": (6.68, -3.81),
    "gym": (2.0, 4.26),
    "lunch": (-0.47, 3.0),
    "art": (0.2, -4.0),
    "bedroom": (-6.47, -0.09),
    "desk": (-8.19, 0.91),
    "room": (-6.0, -2.83),
    "living": (0.0, 0.0),
}

LOCATION_ALIASES = {
    "dinette": "dinette",
    "antecomedor": "dinette",
    "comedor": "dinette",
    "kitchen": "kitchen",
    "cocina": "kitchen",
    "gym": "gym",
    "gimnasio": "gym",
    "lunch": "lunch",
    "desayunador": "lunch",
    "terraza": "lunch",
    "balcon": "lunch",
    "balcón": "lunch",
    "art": "art",
    "arte": "art",
    "cuadros": "art",
    "bedroom": "bedroom",
    "recamara": "bedroom",
    "recámara": "bedroom",
    "dormitorio": "bedroom",
    "habitacion": "bedroom",
    "habitación": "bedroom",
    "desk": "desk",
    "escritorio": "desk",
    "estudio": "desk",
    "oficina": "desk",
    "room": "room",
    "cuarto": "room",
    "vestidor": "room",
    "living": "living",
    "sala": "living",
    "inicio": "living",
    "centro": "living",
}

LOCATION_ES = {
    "dinette": "el antecomedor",
    "kitchen": "la cocina",
    "gym": "el gimnasio",
    "lunch": "el desayunador",
    "art": "la zona de arte",
    "bedroom": "la recámara",
    "desk": "el escritorio",
    "room": "el cuarto",
    "living": "la sala",
}

ARM_POSES = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "navigate": [0.0, -1.66, -0.72, 0.0, 1.17, 0.0],
    "point_front": [0.0, -0.35, -0.65, 0.0, -0.57, 0.0],
    "look_table": [0.0, -0.35, -0.75, 0.0, -0.10, 0.0],
    "look_left": [1.05, -0.35, -0.65, 0.0, -0.35, 0.0],
    "look_right": [-1.05, -0.35, -0.65, 0.0, -0.35, 0.0],
}

OBJECT_SYNONYMS = {
    "ball": {"ball", "pelota", "balon", "balón", "esfera"},
    "barbell": {"barbell", "pesas", "pesa", "mancuerna", "mancuernas", "barra"},
    "bench_press": {"bench_press", "bench press", "banca", "banca de ejercicio", "banca de pesas"},
    "bed": {"bed", "cama", "colchon", "colchón"},
    "box": {"box", "caja", "empaque", "paquete"},
    "fridge": {"fridge", "refrigerator", "refrigerador", "refri", "nevera"},
    "chair": {"chair", "silla", "asiento"},
    "photo": {"photo", "portrait", "cuadro", "retrato", "foto", "fotografia", "fotografía", "pintura"},
    "shoe": {"shoe", "zapato", "zapatos", "tenis", "calzado"},
    "sofa": {"sofa", "couch", "sofá", "sillon", "sillón"},
    "table": {"table", "mesa", "comedor", "escritorio"},
}

MAP_BOUNDS = (-9.3, 9.3, -5.4, 5.5)


class OllamaPlanningNode(Node):
    """NLP orchestrator classifying user intents with Ollama 3 and executing ROS 2 skills."""

    def __init__(self):
        super().__init__("ollama_planning_node")
        self.get_logger().info("INITIALIZING OLLAMA PLANNING NODE - " + NAME)

        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("model_name", "llama3")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("frame_id", "map")

        self.url_api = self.get_parameter("ollama_url").get_parameter_value().string_value
        self.model_name = self.get_parameter("model_name").get_parameter_value().string_value
        goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.system_prompt = self._load_system_prompt()
        self.msg_history = []
        self.prompt = ""
        self.new_prompt = False
        self.tts_busy = False
        self.goal_reached_flag = False
        self.navigating_to = None
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_query = self.create_subscription(
            String, "/sp_rec/recognized", self.callback_prompt, 10
        )
        self.sub_tts_busy = self.create_subscription(
            Bool, "/tts_busy", self.callback_tts_busy, 10
        )
        self.sub_goal_reached = self.create_subscription(
            Bool, "/navigation/goal_reached", self.callback_goal_reached, 10
        )
        self.sub_joint_states = self.create_subscription(
            JointState, "/joint_states", self.callback_joint_states, 10
        )

        self.pub_tts = self.create_publisher(String, "/tts_query", 10)
        self.pub_goal = self.create_publisher(PoseStamped, goal_topic, 10)
        self.pub_arm_traj = self.create_publisher(
            JointTrajectory, "/xarm6_traj_controller/joint_trajectory", 10
        )
        self.pub_gripper_traj = self.create_publisher(
            JointTrajectory, "/xarm_gripper_traj_controller/joint_trajectory", 10
        )

        self.clt_recog_objs = self.create_client(RecognizeObjects, "/vision/recognize_objects")
        self.clt_recog_obj = self.create_client(RecognizeObject, "/vision/recognize_object")
        self.clt_ik = self.create_client(
            InverseKinematicsPose2Pose, "/manipulation/ik_pose2pose"
        )
        self.clt_plan_path = self.create_client(GetPlan, "/path_planning/plan_path")

    def _load_system_prompt(self):
        """Load master system prompt from config/Prompts.txt."""
        try:
            prompts_file = os.path.join(
                get_package_share_directory("llm_planning"), "config", "Prompts.txt"
            )
            with open(prompts_file, "r", encoding="utf-8") as fh:
                content = fh.read().strip()
                if content:
                    return content
        except Exception as exc:
            self.get_logger().warn(f"Could not read Prompts.txt ({exc}); using fallback prompt.")
        return (
            "Eres Thanos, robot de servicio en ROS 2. Clasifica la orden en JSON con llaves: "
            "intent (NAVIGATION|VISION|MANIPULATION|QA|REJECT), location, coordinates, "
            "target_object, arm_action, joint_angles, cartesian_pose, speech (en español)."
        )

    def callback_prompt(self, msg):
        """Queue incoming natural-language text when idle."""
        text = msg.data.strip()
        if not text:
            return
        if self.new_prompt:
            self.get_logger().info("Busy processing previous prompt; ignoring new input.")
            return
        self.prompt = text
        self.new_prompt = True

    def callback_tts_busy(self, msg):
        """Track whether TTS node is currently speaking."""
        self.tts_busy = bool(msg.data)

    def callback_goal_reached(self, msg):
        """Record navigation completion notification."""
        if msg.data:
            self.goal_reached_flag = True

    def callback_joint_states(self, msg):
        """Extract joint1..joint6 angles by name from /joint_states."""
        if not msg.name or len(msg.position) < len(msg.name):
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        updated = []
        for idx in range(1, 7):
            jname = f"joint{idx}"
            if jname not in name_to_pos:
                return
            updated.append(float(name_to_pos[jname]))
        self.current_joints = updated

    def _get_robot_pose(self):
        """Lookup current map->base_link pose if TF is active."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return float(t.transform.translation.x), float(t.transform.translation.y)
        except TransformException:
            return None

    def get_robot_state_context(self):
        """Build concise simulator state string injected into Ollama user prompts."""
        pose = self._get_robot_pose()
        if pose is not None:
            rx, ry = pose
            nearest = min(
                LOCATIONS.items(),
                key=lambda item: math.hypot(item[1][0] - rx, item[1][1] - ry),
            )[0]
            loc_str = f"pose=({rx:.2f}, {ry:.2f}) cerca de {nearest}"
        else:
            loc_str = "ubicación desconocida (TF no disponible)"

        nav_ready = self.pub_goal.get_subscription_count() > 0
        arm_ready = self.pub_arm_traj.get_subscription_count() > 0
        vision_ready = self.clt_recog_objs.service_is_ready()
        return (
            f"{loc_str}; navegacion_activa={nav_ready}; "
            f"brazo_activo={arm_ready}; vision_activa={vision_ready}"
        )

    @staticmethod
    def _parse_json_response(raw_text):
        """Parse Ollama JSON output cleanly, extracting {...} block if wrapped."""
        try:
            data = json.loads(raw_text)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
        match = re.search(r"\{.*\}", raw_text, flags=re.DOTALL)
        if match:
            try:
                data = json.loads(match.group(0))
                if isinstance(data, dict):
                    return data
            except Exception:
                pass
        return None

    def query_ollama(self, user_text, state_context=None):
        """Query local Ollama 3 /api/chat in JSON mode and return parsed intent dict."""
        if state_context is None:
            state_context = self.get_robot_state_context()

        user_content = f"[Estado actual del simulador: {state_context}]\nOrden: {user_text}"
        messages = [{"role": "system", "content": self.system_prompt}]
        messages.extend(self.msg_history[-6:])
        messages.append({"role": "user", "content": user_content})

        payload = {
            "model": self.model_name,
            "messages": messages,
            "format": "json",
            "stream": False,
            "options": {"temperature": 0.1, "num_ctx": 4096},
        }

        try:
            resp = requests.post(self.url_api, json=payload, timeout=30.0)
            resp.raise_for_status()
            raw_content = resp.json()["message"]["content"]
        except Exception as exc:
            self.get_logger().error(f"Ollama API request failed: {exc}")
            return {
                "intent": "REJECT",
                "location": None,
                "coordinates": None,
                "target_object": None,
                "arm_action": None,
                "joint_angles": None,
                "cartesian_pose": None,
                "speech": "No puedo conectarme con mi modelo de lenguaje local en este momento.",
            }

        parsed = self._parse_json_response(raw_content)
        if parsed is None:
            self.get_logger().warn(f"Invalid JSON from Ollama: {raw_content}")
            return {
                "intent": "QA",
                "location": None,
                "coordinates": None,
                "target_object": None,
                "arm_action": None,
                "joint_angles": None,
                "cartesian_pose": None,
                "speech": raw_content[:200] if raw_content else "No logré entender la orden.",
            }

        self.msg_history.append({"role": "user", "content": user_text})
        self.msg_history.append({"role": "assistant", "content": raw_content})
        self.msg_history = self.msg_history[-6:]
        return parsed

    def _speak(self, text):
        """Publish natural-language response to /tts_query."""
        if not text:
            return
        self.get_logger().info(f"TTS -> '{text}'")
        self.pub_tts.publish(String(data=str(text)))

    def _wait_tts_finish(self, timeout_sec=8.0):
        """Allow TTS node to speak without blocking incoming callbacks forever."""
        start = time.monotonic()
        saw_busy = False
        while rclpy.ok() and (time.monotonic() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0)
            if self.tts_busy:
                saw_busy = True
            elif saw_busy:
                break
            elif (time.monotonic() - start) > 0.6 and self.pub_tts.get_subscription_count() == 0:
                break
            elif (time.monotonic() - start) > 1.2 and not saw_busy:
                break
            self.get_clock().sleep_for(Duration(seconds=0.05))

    def _handle_navigation(self, plan_dict):
        """Validate navigation target against simulator state and publish /goal_pose."""
        if self.pub_goal.get_subscription_count() == 0:
            return "No puedo navegar porque el sistema de navegación del simulador no está activo."

        loc_raw = plan_dict.get("location")
        coords = plan_dict.get("coordinates")

        target_name = None
        gx, gy = None, None

        if isinstance(loc_raw, str) and loc_raw.strip():
            canonical = LOCATION_ALIASES.get(loc_raw.strip().lower())
            if canonical and canonical in LOCATIONS:
                target_name = LOCATION_ES.get(canonical, canonical)
                gx, gy = LOCATIONS[canonical]
            else:
                return f"No conozco la ubicación {loc_raw} dentro del departamento."
        elif isinstance(coords, (list, tuple)) and len(coords) >= 2:
            try:
                gx, gy = float(coords[0]), float(coords[1])
                target_name = f"las coordenadas {gx:.2f}, {gy:.2f}"
            except (TypeError, ValueError):
                return "Las coordenadas indicadas no son válidas."
        else:
            return plan_dict.get("speech") or "No entendí a qué lugar del departamento debo ir."

        min_x, max_x, min_y, max_y = MAP_BOUNDS
        if not (min_x <= gx <= max_x and min_y <= gy <= max_y):
            return f"No puedo ir a {target_name} porque está fuera de los límites del departamento."

        # If path planner and TF are running, validate that an actual obstacle-free path exists
        pose = self._get_robot_pose()
        if pose is not None and self.clt_plan_path.wait_for_service(timeout_sec=0.3):
            req = GetPlan.Request()
            req.start.pose.position.x = pose[0]
            req.start.pose.position.y = pose[1]
            req.goal.pose.position.x = gx
            req.goal.pose.position.y = gy
            future = self.clt_plan_path.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                if len(future.result().plan.poses) == 0:
                    return (
                        f"No puedo navegar hacia {target_name} porque no hay una ruta libre de "
                        "obstáculos hasta ese punto."
                    )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.navigating_to = target_name
        self.goal_reached_flag = False
        self.get_logger().info(f"Navigation goal published -> {target_name} ({gx:.2f}, {gy:.2f})")
        return plan_dict.get("speech") or f"Voy en camino hacia {target_name}."

    def _handle_vision(self, plan_dict):
        """Call /vision/recognize_objects and formulate spoken summary."""
        if not self.clt_recog_objs.wait_for_service(timeout_sec=1.0):
            return "No puedo usar la cámara porque el nodo de visión artificial no está activo."

        req = RecognizeObjects.Request()
        future = self.clt_recog_objs.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            return "El sistema de visión tardó demasiado o no tiene imagen de cámara disponible."

        objs = list(future.result().objs)
        target = plan_dict.get("target_object")
        if not objs:
            if target:
                return f"No detecto ningún objeto ni {target} frente a mi cámara en este momento."
            return "No detecto objetos frente a mi cámara en este momento."

        if isinstance(target, str) and target.strip() and target.strip().lower() not in ("all", "any", "todos"):
            t_low = target.strip().lower()
            syn_group = {t_low}
            for base_cls, words in OBJECT_SYNONYMS.items():
                if t_low in words or base_cls in t_low:
                    syn_group.update(words)
                    syn_group.add(base_cls)

            matched = [
                o
                for o in objs
                if any(
                    w in o.id.lower() or w in o.category.lower() or o.id.lower() in w or o.category.lower() in w
                    for w in syn_group
                )
            ]
            if matched:
                best = max(matched, key=lambda o: o.confidence)
                dist = math.sqrt(
                    best.pose.position.x ** 2 + best.pose.position.y ** 2 + best.pose.position.z ** 2
                )
                dist_txt = f" a unos {dist:.1f} metros" if dist > 0.05 else ""
                if len(matched) > 1:
                    return f"Sí, detecto {len(matched)} {best.category}{dist_txt}."
                return f"Sí, detecto {best.category}{dist_txt}."

            seen_cats = sorted({o.category or o.id for o in objs})
            return (
                f"No veo {target} frente a mí. Por ahora solo detecto: "
                + ", ".join(seen_cats[:5])
                + "."
            )

        counts = {}
        closest_dist = None
        for o in objs:
            label = o.category or o.id
            counts[label] = counts.get(label, 0) + 1
            d = math.sqrt(o.pose.position.x ** 2 + o.pose.position.y ** 2 + o.pose.position.z ** 2)
            if d > 0.05 and (closest_dist is None or d < closest_dist):
                closest_dist = d

        parts = [f"{cnt} {name}" if cnt > 1 else name for name, cnt in counts.items()]
        summary = ", ".join(parts)
        if closest_dist is not None:
            return f"Frente a mí veo {summary}; el más cercano está a {closest_dist:.1f} metros."
        return f"Frente a mí veo {summary}."

    def _publish_arm_points(self, points_list):
        """Publish single or multi-waypoint JointTrajectory to /xarm6_traj_controller."""
        msg = JointTrajectory()
        # Use stamp=0 so controller executes immediately even with Gazebo sim_time
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        for t_sec, q in points_list:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            sec = int(t_sec)
            nsec = int((t_sec - sec) * 1e9)
            pt.time_from_start.sec = sec
            pt.time_from_start.nanosec = nsec
            msg.points.append(pt)
        self.pub_arm_traj.publish(msg)

    def _handle_manipulation(self, plan_dict):
        """Validate manipulator/gripper action and publish trajectory or call IK."""
        action = (plan_dict.get("arm_action") or "").strip().lower()

        if action in ("open_gripper", "close_gripper"):
            if self.pub_gripper_traj.get_subscription_count() == 0:
                return "No puedo mover la pinza porque el controlador del gripper no está activo."
            msg = JointTrajectory()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.joint_names = ["drive_joint"]
            pt = JointTrajectoryPoint()
            pt.positions = [0.0] if action == "open_gripper" else [0.80]
            pt.time_from_start.sec = 1
            msg.points.append(pt)
            self.pub_gripper_traj.publish(msg)
            return plan_dict.get("speech") or (
                "Abriendo la pinza." if action == "open_gripper" else "Cerrando la pinza."
            )

        if self.pub_arm_traj.get_subscription_count() == 0:
            return "No puedo mover el brazo porque el controlador del manipulador no está activo."

        if action == "wave":
            self._publish_arm_points([
                (1.2, [0.0, -0.30, -0.90, 0.0, 0.50, 0.0]),
                (1.9, [0.0, -0.30, -0.90, 0.65, 0.50, 0.50]),
                (2.6, [0.0, -0.30, -0.90, -0.65, 0.50, -0.50]),
                (3.3, [0.0, -0.30, -0.90, 0.65, 0.50, 0.50]),
                (4.6, ARM_POSES["navigate"]),
            ])
            return plan_dict.get("speech") or "Con gusto, saludando con el brazo."

        if action in ARM_POSES:
            self._publish_arm_points([(1.8, ARM_POSES[action])])
            return plan_dict.get("speech") or f"Moviendo el brazo a posición {action}."

        if action == "joints":
            q = plan_dict.get("joint_angles")
            if not isinstance(q, (list, tuple)) or len(q) != 6:
                return "Necesito seis ángulos en radianes para mover el brazo."
            try:
                q_vals = [float(v) for v in q]
            except (TypeError, ValueError):
                return "Los ángulos articulares indicados no son válidos."
            if any(abs(v) > 3.1416 for v in q_vals):
                return "No puedo mover el brazo porque alguno de los ángulos excede los límites seguros."
            self._publish_arm_points([(2.0, q_vals)])
            return plan_dict.get("speech") or "Moviendo el manipulador a los ángulos indicados."

        if action == "cartesian":
            pose = plan_dict.get("cartesian_pose")
            if not isinstance(pose, (list, tuple)) or len(pose) != 6:
                return "Necesito seis valores cartesianos para calcular la cinemática inversa."
            if not self.clt_ik.wait_for_service(timeout_sec=1.0):
                return "El servicio de cinemática inversa no está disponible en este momento."
            req = InverseKinematicsPose2Pose.Request()
            req.x, req.y, req.z, req.roll, req.pitch, req.yaw = [float(v) for v in pose]
            req.initial_guess = [float(v) for v in self.current_joints]
            future = self.clt_ik.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.done() or future.result() is None or len(future.result().q) != 6:
                return "No puedo alcanzar esa posición cartesiana con el brazo."
            self._publish_arm_points([(2.0, list(future.result().q))])
            return plan_dict.get("speech") or "Moviendo el brazo a la posición solicitada."

        return plan_dict.get("speech") or "No reconocí la acción de manipulación solicitada."

    def execute_intent(self, plan_dict):
        """Dispatch classified intent to ROS 2 subsystems and return spoken response."""
        intent = str(plan_dict.get("intent") or "QA").strip().upper()
        self.get_logger().info(f"Intent={intent} | Plan={plan_dict}")

        if intent == "NAVIGATION":
            return self._handle_navigation(plan_dict)
        if intent == "VISION":
            return self._handle_vision(plan_dict)
        if intent == "MANIPULATION":
            return self._handle_manipulation(plan_dict)
        return plan_dict.get("speech") or "Entendido."

    def spin(self):
        """Main event loop polling user prompts and navigation arrival status."""
        self.get_logger().info("Orchestrator ready. Waiting for commands on /sp_rec/recognized...")
        while rclpy.ok():
            if self.goal_reached_flag and self.navigating_to:
                dest = self.navigating_to
                self.goal_reached_flag = False
                self.navigating_to = None
                self._speak(f"Ya llegué a {dest}.")
                self._wait_tts_finish()

            if self.new_prompt:
                user_text = self.prompt
                self.get_logger().info(f"Processing user input: '{user_text}'")
                state_ctx = self.get_robot_state_context()
                plan_dict = self.query_ollama(user_text, state_ctx)
                spoken_reply = self.execute_intent(plan_dict)
                self._speak(spoken_reply)
                self._wait_tts_finish()
                self.new_prompt = False
                self.get_logger().info("Waiting for next prompt...")

            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.05))


def main(args=None):
    rclpy.init(args=args)
    ollama_planning_node = OllamaPlanningNode()
    try:
        ollama_planning_node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ollama_planning_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
