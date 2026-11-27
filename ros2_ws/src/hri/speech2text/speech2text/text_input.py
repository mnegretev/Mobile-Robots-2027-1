#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# Fallback Text Input Node for HRI Testing without Microphone
#

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"


class TextInputNode(Node):
    """CLI fallback node publishing user commands to /sp_rec/recognized."""

    def __init__(self):
        super().__init__("text_input_node")
        self.get_logger().info("INITIALIZING TEXT INPUT FALLBACK NODE - " + NAME)
        self.pub_recognized = self.create_publisher(String, "/sp_rec/recognized", 10)
        self.sub_tts = self.create_subscription(
            String, "/tts_query", self.callback_tts_query, 10
        )

    def callback_tts_query(self, msg):
        """Display robot spoken responses on console."""
        print(f"\n[Robot Voz]: {msg.data}")
        print("Comando > ", end="", flush=True)

    def cli_loop(self):
        """Read natural-language commands from stdin and publish to /sp_rec/recognized."""
        print("Modo texto activo. Escribe comandos en lenguaje natural ('exit' para terminar).")
        while rclpy.ok():
            try:
                user_input = input("\nHabla > ").strip()
            except EOFError:
                break
            if not user_input:
                continue
            if user_input.lower() in ("salir", "exit", "quit"):
                break
            self.pub_recognized.publish(String(data=user_input))


def main(args=None):
    rclpy.init(args=args)
    node = TextInputNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.cli_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
