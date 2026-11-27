#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# Dataset Image Capture Utility for Custom YOLOv8 Training
#

import os
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from obj_recognizer.obj_recognizer import VenvCompatibleCvBridge

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"


class CaptureDatasetNode(Node):
    """Saves RGB frames from /camera/color/image_raw for custom YOLOv8 labeling."""

    def __init__(self):
        super().__init__("capture_dataset_node")
        self.get_logger().info("INITIALIZING DATASET CAPTURE NODE - " + NAME)

        default_dir = os.path.expanduser("~/Mobile-Robots-2027-1/dataset_images")
        self.declare_parameter("output_dir", default_dir)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("interval_sec", 0.0)

        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.interval_sec = self.get_parameter("interval_sec").get_parameter_value().double_value

        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = VenvCompatibleCvBridge()
        self.latest_msg = None
        self.count = 0

        self.sub_img = self.create_subscription(Image, image_topic, self.callback_img, 1)
        if self.interval_sec > 0.0:
            self.timer = self.create_timer(self.interval_sec, self.save_current_frame)

    def callback_img(self, msg):
        """Store latest camera frame."""
        self.latest_msg = msg

    def save_current_frame(self):
        """Save latest frame to output_dir as JPEG."""
        if self.latest_msg is None:
            self.get_logger().warn("No image received yet on camera topic.")
            return False
        img_bgr = self.bridge.imgmsg_to_cv2(self.latest_msg, desired_encoding="bgr8")
        self.count += 1
        filename = f"frame_{int(time.time() * 1000)}_{self.count:04d}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, img_bgr)
        self.get_logger().info(f"Saved {filepath}")
        return True

    def cli_loop(self):
        """Interactive capture loop when interval_sec == 0."""
        if self.interval_sec > 0.0:
            while rclpy.ok():
                time.sleep(0.5)
            return
        print(f"Guardando imágenes en: {self.output_dir}")
        print("Presiona ENTER para capturar un frame (o escribe 'salir'):")
        while rclpy.ok():
            try:
                cmd = input().strip().lower()
            except EOFError:
                break
            if cmd in ("salir", "exit", "quit"):
                break
            self.save_current_frame()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureDatasetNode()
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
