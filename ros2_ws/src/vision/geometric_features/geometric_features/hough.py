import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2
import math

FULL_NAME = "Abraham Pérez Rojo"

class HoughNode(Node):
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_hough  = img_bgr.copy()
        
        # Conversión a escala de grises y detección de bordes
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_bin = cv2.Canny(img_gray, self.canny_lower, self.canny_upper)
        
        # Medición de tiempo y Transformada de Hough
        start_time = self.get_clock().now()
        lines = cv2.HoughLines(img_bin, self.rho, self.theta, self.hough_threshold)
        end_time = self.get_clock().now()
        
        time_diff = end_time - start_time
        time_ms = time_diff.nanoseconds / 1000000.0
        
        # Dibujo de líneas
        if lines is not None:
            for i in range(0, len(lines)):
                rho_val = lines[i][0][0]
                theta_val = lines[i][0][1]
                a = math.cos(theta_val)
                b = math.sin(theta_val)
                x0 = a * rho_val
                y0 = b * rho_val
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(img_hough, pt1, pt2, (0,0,255), 2, cv2.LINE_AA)
                
        # Mostrar el tiempo
        cv2.putText(img_hough, f"Time [ms]: {time_ms:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Houhgh", img_hough)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING HOUGH NODE - ", FULL_NAME)
        super().__init__("hough_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
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