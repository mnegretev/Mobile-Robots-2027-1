#!/usr/bin/env python3

import rclpy
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64

class CarDriver(Node):
    def __init__(self):
        super().__init__('car_driver_node')

        self.camera_sub = self.create_subscription(Image, '/car/camera/image_color', self.camera_callback, 10)
        self.get_logger().info("car_driver_node is Ready......")

    def camera_callback(self, msg):
        self.get_logger().info(f'¡Datos recibidos! Cámara: {msg.width}x{msg.height}px')


def main(args=None):
    rclpy.init(args=args)
    node = CarDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main