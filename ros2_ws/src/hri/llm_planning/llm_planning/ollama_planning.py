import os
import threading
import requests
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class OllamaPlanningNode(Node):
    def load_prompts(self, path):
        with open(path, "r", encoding="utf-8") as f:
            return [l.rstrip() for l in f.readlines() if len(l.rstrip()) > 3]

    def send_prompt(self, msg):
        self.msg_history.append({"role": "user", "content": msg})
        try:
            resp = requests.post(
                self.url_api,
                json={
                    "model": "llama3",
                    "messages": self.msg_history,
                    "stream": False,
                    "keep_alive": "15m",
                    "options": {
                        "num_ctx": 2048,
                        "num_predict": 65,
                        "temperature": 0.4,
                    },
                },
                timeout=120,
            )
            resp.raise_for_status()
            data = resp.json()
            if "message" not in data:
                self.get_logger().error(f"Respuesta sin 'message': {data}")
                self.msg_history.pop()
                return False
            self.msg_history.append(data["message"])
            return True
        except Exception as e:
            self.get_logger().error(f"Error con Ollama: {e}")
            self.msg_history.pop()
            return False

    def callback_prompt(self, msg):
        if self.new_prompt or self.busy:
            return
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"[Voz /sp_rec/recognized]: {text}")
            self.prompt = text
            self.new_prompt = True

    def _stdin_loop(self):
        # Entrada opcional por teclado para pruebas rápidas sin micrófono (no publica en /sp_rec/recognized)
        while rclpy.ok():
            try:
                user_text = input().strip()
            except EOFError:
                break
            if not user_text:
                continue
            if self.new_prompt or self.busy:
                print("[Espera a que termine la respuesta actual...]", flush=True)
                continue
            self.prompt = user_text
            self.new_prompt = True

    # --- Habilidades del robot simulado (conecta aquí los tópicos/acciones de tu simulador) ---

    def execute_navigate(self, target: str) -> str:
        # Coordenadas de ejemplo para puntos comunes; ajusta según tu mapa en simulación
        waypoints = {
            "cocina": (2.0, 1.0, 0.0),
            "mesa": (1.5, -0.5, 0.0),
            "sala": (0.0, 2.0, 1.57),
            "inicio": (0.0, 0.0, 0.0),
            "home": (0.0, 0.0, 0.0),
        }
        key = target.lower().strip()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        if key in waypoints:
            x, y, yaw = waypoints[key]
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            self.pub_goal.publish(pose)
            self.get_logger().info(f"[SIM NAV] Navegando a '{target}' ({x}, {y})")
            return f"Navegando hacia {target}."

        self.pub_nav_cmd.publish(String(data=target))
        self.get_logger().info(f"[SIM NAV] Enviando destino '{target}' a /sim/navigate_to")
        return f"Moviéndome hacia {target}."

    def execute_detect(self, obj_name: str) -> str:
        self.pub_detect_cmd.publish(String(data=obj_name))
        self.get_logger().info(f"[SIM VISION] Buscando objeto '{obj_name}'")
        return f"Buscando {obj_name} con la cámara."

    def execute_arm(self, pose_or_action: str) -> str:
        self.pub_arm_cmd.publish(String(data=pose_or_action))
        self.get_logger().info(f"[SIM ARM] Moviendo manipulador: '{pose_or_action}'")
        return f"Moviendo el brazo a {pose_or_action}."

    def process_reply_and_speak(self, reply: str):
        spoken_parts = []

        for raw_line in reply.splitlines():
            line = raw_line.strip().lstrip("-* ").strip("`")
            if not line:
                continue

            upper = line.upper()
            if upper.startswith("NAVIGATE:"):
                target = line.split(":", 1)[1].strip()
                spoken_parts.append(self.execute_navigate(target))
            elif upper.startswith("DETECT:"):
                obj_name = line.split(":", 1)[1].strip()
                spoken_parts.append(self.execute_detect(obj_name))
            elif upper.startswith("ARM:"):
                action = line.split(":", 1)[1].strip()
                spoken_parts.append(self.execute_arm(action))
            else:
                spoken_parts.append(line)

        final_speech = " ".join(spoken_parts).strip()
        if not final_speech:
            final_speech = "Listo."

        self.get_logger().info(f"[TTS -> /tts_query]: {final_speech}")
        self.pub_tts.publish(String(data=final_speech))
        return final_speech

    def __init__(self):
        super().__init__("ollama_planning_node")
        self.get_logger().info("INITIALIZING OLLAMA PLANNING NODE (HRI + SIM)")
        self.msg_history = []
        self.url_api = "http://localhost:11434/api/chat"
        self.prompt = ""
        self.new_prompt = False
        self.busy = False

        # Voz: entrada desde speech2text, salida hacia text2speech
        self.sub_query = self.create_subscription(String, "/sp_rec/recognized", self.callback_prompt, 10)
        self.pub_tts = self.create_publisher(String, "/tts_query", 10)

        # Tópicos de control para el simulador (adapta según tu paquete de simulación)
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_nav_cmd = self.create_publisher(String, "/sim/navigate_to", 10)
        self.pub_detect_cmd = self.create_publisher(String, "/sim/detect_object", 10)
        self.pub_arm_cmd = self.create_publisher(String, "/sim/arm_command", 10)

        self.stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self.stdin_thread.start()

    def spin(self):
        extra_prompts = []
        prompts_file = os.path.join(get_package_share_directory("llm_planning"), "config", "Prompts.txt")
        if os.path.exists(prompts_file):
            extra_prompts = self.load_prompts(prompts_file)

        system_prompt = (
            "Eres un robot de servicio simulado en Ubuntu 24.04 con ROS 2 Jazzy. "
            "Interactúas por voz: responde en español natural, claro y breve (máximo 20 palabras), "
            "sin usar emojis, asteriscos ni viñetas para que text2speech lo lea bien.\n"
            "Tus ÚNICAS habilidades físicas/sensoriales son:\n"
            "1) Navegar a un punto -> agrega línea exacta: NAVIGATE: <punto>\n"
            "2) Reconocer un objeto -> agrega línea exacta: DETECT: <objeto>\n"
            "3) Mover el manipulador -> agrega línea exacta: ARM: <posicion_o_accion>\n"
            "4) Contestar preguntas -> responde directo en texto.\n"
            "Si te piden algo que NO puedes hacer (volar, cocinar, limpiar, saltar, etc.), "
            "NO emitas NAVIGATE/DETECT/ARM y explica con lenguaje natural y breve que no puedes realizar esa tarea."
        )
        if extra_prompts:
            system_prompt += "\nContexto adicional:\n" + "\n".join(extra_prompts)

        self.msg_history.append({"role": "system", "content": system_prompt})
        self.get_logger().info("Listo para recibir órdenes por voz (/sp_rec/recognized) o texto aquí.")

        while rclpy.ok():
            if self.new_prompt and not self.busy:
                self.busy = True
                self.new_prompt = False
                current_prompt = self.prompt

                if self.send_prompt(current_prompt):
                    full_reply = self.msg_history[-1]["content"]
                    self.get_logger().info(f"LLM raw reply:\n{full_reply}")
                    spoken = self.process_reply_and_speak(full_reply)

                    # Tiempo de espera proporcional a la frase para que t2s termine de hablar
                    delay_counter = int(1.4 * len(spoken) + 15)
                    while delay_counter > 0 and rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0)
                        self.get_clock().sleep_for(Duration(seconds=0.05))
                        delay_counter -= 1

                self.busy = False

            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.05))


def main(args=None):
    rclpy.init(args=args)
    node = OllamaPlanningNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()