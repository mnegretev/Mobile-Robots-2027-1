import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2
import math

FULL_NAME = "Abraham Pérez Rojo"

class HoughPNode(Node):
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_houghP = img_bgr.copy()
        
        # Conversión a escala de grises y detección de bordes
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_bin = cv2.Canny(img_gray, self.canny_lower, self.canny_upper)
        
        # Medición de tiempo y Transformada de Hough Probabilística
        start_time = self.get_clock().now()
        linesP = cv2.HoughLinesP(img_bin, self.rho, self.theta, self.hough_threshold, None, self.min_length, self.max_gap)
        end_time = self.get_clock().now()
        
        time_diff = end_time - start_time
        time_ms = time_diff.nanoseconds / 1000000.0
        
        # Dibujo de líneas
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(img_houghP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)
                
        # Mostrar el tiempo
        cv2.putText(img_houghP, f"Time [ms]: {time_ms:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Houhgh P", img_houghP)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING PROBABILISTIC HOUGH NODE - ", FULL_NAME)
        super().__init__("houghP_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
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