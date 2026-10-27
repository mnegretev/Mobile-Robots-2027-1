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

FULL_NAME = "Enrique Medrano Solano"

class HoughNode(Node):
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_hough  = img_bgr.copy()
        #
        # TODO:
        # Change the color space of the image 'img_bgr' to grayscale
        # Get borders using the Canny edge detector.
        # From boders, get lines using the cv2.HoughLines function
        # Draw the resulting lines over the image img_hough
        # Check the online tutorials for OpenCV documentation
        # https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        # https://docs.opencv.org/4.x/d6/d6e/group__imgproc__draw.html
        # Measure the time used to detect lines (only line detection)
        # Check the use of node.get_clock().now() function.
        # Display the processing time on the img_houghP image
        # Use the parameters self.canny_lower, self.canny_upper, self.rho, self.theta,
        # and self.hough_threshold
        #
        start_time = self.get_clock().now()
        #Deteccipón de bordes
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (5,5), 0)
        edges = cv2.Canny(img_blur, self.canny_lower, self.canny_upper)

        lines = cv2.HoughLines(edges, self.rho, self.theta, self.hough_threshold)
        end_time = self.get_clock().now()
        time_ms = (end_time - start_time).nanoseconds / 1e6

        #Lineas resultantes
        if lines is not None:
            for line in lines:
                rho_val, theta_val = line[0]
                a = math.cos(theta_val)
                b = math.sin(theta_val)
                x0 = a * rho_val
                y0 = b * rho_val

                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(img_hough, pt1, pt2, (0,0,255), 2)

        # Desplegar el tiempo en la imagen
        cv2.putText(img_hough, f"Tiempo: {time_ms:.2f} ms", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
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
