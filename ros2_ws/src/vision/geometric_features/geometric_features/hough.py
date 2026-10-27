#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# HOUGH LINE DETECTOR
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import time
from rclpy.qos import qos_profile_sensor_data

FULL_NAME = "LEONARDO ALEJANDRO GARCIA LERMA"

class HoughNode(Node):
    def callback_img(self, msg):
        # Leer parámetros en tiempo real para evitar recompilar
        canny_l = self.get_parameter("canny_l").value
        canny_u = self.get_parameter("canny_u").value
        rho = self.get_parameter("rho").value
        theta = self.get_parameter("theta").value
        hough_th = self.get_parameter("hough_th").value

        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_hough  = img_bgr.copy()
        
        # Iniciar medición de tiempo
        start_time = time.time()
        
        # 1. Convertir a escala de grises y aplicar Canny
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, canny_l, canny_u)
        
        # 2. Detectar líneas con Transformada de Hough Estándar
        lines = cv2.HoughLines(edges, rho, theta, hough_th)
        
        # 3. Dibujar las líneas sobre la imagen img_hough
        if lines is not None:
            for line in lines:
                r, th = line[0]
                a = math.cos(th)
                b = math.sin(th)
                x0 = a * r
                y0 = b * r
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(img_hough, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # 4. Medir tiempo de ejecución y desplegarlo
        elapsed_ms = (time.time() - start_time) * 1000.0
        cv2.putText(img_hough, f"time[ms]={elapsed_ms:.2f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Houhgh", img_hough)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING HOUGH NODE - ", FULL_NAME)
        super().__init__("hough_node")
        self.br = CvBridge()
        # Conexión corregida para evitar bloqueos del simulador
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, qos_profile_sensor_data)
        
        self.declare_parameter("canny_l", 10)
        self.declare_parameter("canny_u", 20)
        self.declare_parameter("rho", 5)
        self.declare_parameter("theta", 0.1)
        self.declare_parameter("hough_th", 50)
        print("Node ready. Awaiting images...")

def main(args=None):
    rclpy.init(args=args)
    hough_node = HoughNode()
    rclpy.spin(hough_node)
    hough_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()