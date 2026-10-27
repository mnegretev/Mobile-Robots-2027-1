#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# PROBABILISTIC HOUGH LINE DETECTOR
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

class HoughPNode(Node):
    def callback_img(self, msg):
        # Leer parámetros en tiempo real para evitar recompilar
        canny_l = self.get_parameter("canny_l").value
        canny_u = self.get_parameter("canny_u").value
        rho = self.get_parameter("rho").value
        theta = self.get_parameter("theta").value
        hough_th = self.get_parameter("hough_th").value
        min_len = self.get_parameter("min_len").value
        max_gap = self.get_parameter("max_gap").value

        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_houghP = img_bgr.copy()
        
        # Iniciar medición de tiempo (solo detección de líneas)
        start_time = time.time()
        
        # 1. Convertir a escala de grises, aplicar suavizado y Canny
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, canny_l, canny_u)
        
        # 2. Detectar líneas con Transformada Hough Probabilística
        lines = cv2.HoughLinesP(edges, rho, theta, hough_th, minLineLength=min_len, maxLineGap=max_gap)
        
        # 3. Dibujar las líneas resultantes sobre la imagen img_houghP
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_houghP, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # 4. Medir tiempo de ejecución y desplegarlo en la imagen
        elapsed_ms = (time.time() - start_time) * 1000.0
        cv2.putText(img_houghP, f"time[ms]={elapsed_ms:.2f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Houhgh P", img_houghP)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING PROBABILISTIC HOUGH NODE - ", FULL_NAME)
        super().__init__("houghP_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, qos_profile_sensor_data)
        
        self.declare_parameter("canny_l", 10)
        self.declare_parameter("canny_u", 20)
        self.declare_parameter("rho", 5)
        self.declare_parameter("theta", 0.1)
        self.declare_parameter("hough_th", 50)
        self.declare_parameter("min_len", 30)
        self.declare_parameter("max_gap", 10)
        print("Node ready. Awaiting images...")

def main(args=None):
    rclpy.init(args=args)
    houghP_node = HoughPNode()
    rclpy.spin(houghP_node)
    houghP_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()