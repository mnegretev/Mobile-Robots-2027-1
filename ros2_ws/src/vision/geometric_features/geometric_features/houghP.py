#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# PROBABILISTIC HOUGH LINE DETECTOR
#
# Instructions:
# Complete the code necessary to implement the probabilistic Hough line
# detector using the corresponding OpenCV function
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2
import math

FULL_NAME = "Omar Rosario Carrasco"

class HoughPNode(Node):
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_houghP = img_bgr.copy()
        #
        # TODO:
        # Change the color space of the image 'img_bgr' to grayscale
        # Get borders using the Canny edge detector.
        # From boders, get lines using the cv2.HoughLinesP function
        # Draw the resulting lines over the image img_houghP
        # Check the online tutorials for OpenCV documentation
        # https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        # https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html
        # Measure the time used to detect lines (only line detection)
        # Check the use of node.get_clock().now() function.
        # Display the processing time on the img_houghP image
        # Use the parameters self.canny_lower, self.canny_upper, self.rho, self.theta,
        # self.hough_threshold, self.min_length and self.max_gap
        #
        # Escala de grises y bordes con Canny: la imagen binaria de bordes es la
        # entrada de la Transformada Hough (solo votan los píxeles de borde)
        img_gray  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_edges = cv2.Canny(img_gray, self.canny_lower, self.canny_upper)

        # Solo se mide la detección de líneas. HoughLinesP toma píxeles de borde al
        # azar; cuando una celda del acumulador alcanza hough_threshold recorre la
        # recta sobre los bordes (saltando huecos <= max_gap), guarda el segmento si
        # mide >= min_length y retira esos píxeles para que ya no voten.
        # minLineLength y maxLineGap van con nombre: el 5o argumento posicional es 'lines'.
        t_start = self.get_clock().now()
        lines = cv2.HoughLinesP(img_edges, self.rho, self.theta, self.hough_threshold,
                                minLineLength=self.min_length, maxLineGap=self.max_gap)
        t_ms = (self.get_clock().now() - t_start).nanoseconds / 1e6

        # Cada segmento llega directamente como sus extremos (x1, y1, x2, y2).
        # reshape(-1, 4): OpenCV 4.x devuelve forma (N,1,4) y OpenCV 5.x (N,4)
        n_lines = 0
        if lines is not None:
            n_lines = len(lines)
            for x1, y1, x2, y2 in lines.reshape(-1, 4):
                cv2.line(img_houghP, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2, cv2.LINE_AA)

        # Ubuntu no es de tiempo real: además del tiempo del cuadro se acumula el
        # promedio de todos los cuadros (para la tabla de tiempos del reporte)
        self.times.append(t_ms)
        t_avg = numpy.mean(self.times)
        cv2.putText(img_houghP, "time[ms]=%.2f" % t_ms, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img_houghP, "avg[ms]=%.2f  segments=%d" % (t_avg, n_lines), (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if len(self.times) % 30 == 0:
            self.get_logger().info("frames=%d  t_avg=%.3f ms  t_std=%.3f ms  segments=%d"
                                   % (len(self.times), t_avg, numpy.std(self.times), n_lines))

        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Houhgh P", img_houghP)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING PROBABILISTIC HOUGH NODE - ", FULL_NAME)
        super().__init__("houghP_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.callback_img, 1)
        # En house_simul (realsense:=True) el bridge de Gazebo publica la imagen RGB en
        # /camera/color/image_raw (ver mbot_demo/config/gz_bridge.yaml), no en /camera/image_raw.
        # Se escuchan ambos tópicos para que el nodo reciba imágenes en cualquiera de los dos casos.
        self.sub_img_color = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        self.times = []    # Tiempos de detección [ms] de cada cuadro, para promediar
        self.declare_parameter("canny_l",10)
        self.declare_parameter("canny_u",20)
        self.declare_parameter("rho", 10)
        self.declare_parameter("theta", 0.1)
        self.declare_parameter("hough_th", 20)
        self.declare_parameter("min_len",1)
        self.declare_parameter("max_gap",1)
        self.canny_lower = self.get_parameter("canny_l").get_parameter_value().integer_value
        self.canny_upper = self.get_parameter("canny_u").get_parameter_value().integer_value
        self.rho   = self.get_parameter("rho").get_parameter_value().integer_value
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.hough_threshold = self.get_parameter("hough_th").get_parameter_value().integer_value
        self.min_length = self.get_parameter("min_len").get_parameter_value().integer_value
        self.max_gap = self.get_parameter("max_gap").get_parameter_value().integer_value
        print("Starting line detection with parameters: ")
        print("[Canny lower, Canny upper]=", [self.canny_lower, self.canny_upper])
        print("[rho, theta, Hough threshold]=", [self.rho, self.theta, self.hough_threshold])
        print("[min length, max gap]=",[self.min_length, self.max_gap])

def main(args=None):
    rclpy.init(args=args)
    houghP_node = HoughPNode()
    rclpy.spin(houghP_node)
    houghP_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
