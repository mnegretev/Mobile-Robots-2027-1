#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# HOUGH LINE DETECTOR
#
# Instructions:
# Complete the code necessary to implement the Hough line
# detector using the corresponding OpenCV function
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2
import math

FULL_NAME = "SOLORIO GONZALEZ ALDO BRUNO"

class HoughNode(Node):
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_hough = img_bgr.copy()

        # Convertir a grises, suavizar y detectar bordes.
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (15, 15), 0)
        img_bin = cv2.Canny(
            img_blur,
            self.canny_lower,
            self.canny_upper
        )

        # Medir solamente la deteccion de lineas.
        t_start = self.get_clock().now()

        lines = cv2.HoughLines(
            img_bin,
            self.rho,
            self.theta,
            self.hough_threshold
        )

        t_ms = (self.get_clock().now() - t_start).nanoseconds / 1e6

        # Dibujar cada recta a partir de rho y theta.
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = math.cos(theta)
                b = math.sin(theta)

                x0 = a * rho
                y0 = b * rho

                x1 = int(round(x0 - 1000 * b))
                y1 = int(round(y0 + 1000 * a))
                x2 = int(round(x0 + 1000 * b))
                y2 = int(round(y0 - 1000 * a))

                cv2.line(
                    img_hough,
                    (x1, y1),
                    (x2, y2),
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA
                )

        # Promediar bloques de 100 mediciones.
        self.tiempos_ms.append(t_ms)

        if len(self.tiempos_ms) == 100:
            promedio = sum(self.tiempos_ms) / len(self.tiempos_ms)
            self.get_logger().info(
                f"Hough: promedio de 100 cuadros = {promedio:.3f} ms"
            )
            self.tiempos_ms.clear()

        # Mostrar el tiempo del cuadro actual.
        texto = f"Hough: {t_ms:.3f} ms"

        cv2.putText(
            img_hough, texto, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 0, 0), 4, cv2.LINE_AA
        )
        cv2.putText(
            img_hough, texto, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0), 2, cv2.LINE_AA
        )

        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Hough", img_hough)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING HOUGH NODE - ", FULL_NAME)
        super().__init__("hough_node")
        self.br = CvBridge()
        self.tiempos_ms = []
        self.sub_img = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_img,
            1
        )
        self.declare_parameter("canny_l",10)
        self.declare_parameter("canny_u",20)
        self.declare_parameter("rho", 10)
        self.declare_parameter("theta", 0.1)
        self.declare_parameter("hough_th", 20)
        self.canny_lower = self.get_parameter("canny_l").get_parameter_value().integer_value
        self.canny_upper = self.get_parameter("canny_u").get_parameter_value().integer_value
        self.rho   = self.get_parameter("rho").get_parameter_value().integer_value
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.hough_threshold = self.get_parameter("hough_th").get_parameter_value().integer_value
        print("Starting line detection with parameters: ")
        print("[Canny lower, Canny upper]=", [self.canny_lower, self.canny_upper])
        print("[rho, theta, Hough threshold]=", [self.rho, self.theta, self.hough_threshold])

def main(args=None):
    rclpy.init(args=args)
    hough_node = HoughNode()
    rclpy.spin(hough_node)
    hough_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
