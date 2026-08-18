#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# HARRIS CORNER DETECTOR
#
# Instructions:
# Complete the code necessary to implement the Harris corner detector
# using the function provided by OpenCV
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2

FULL_NAME = "FULL NAME"

class HarrisNode(Node):
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_cor = img_bgr.copy()
        #
        # TODO:
        # Change the color space of the image 'img_bgr' to grayscale
        # Get edges using the cv2.Canny function, use as parameters
        # the variables self.canny_lower and self.canny_upper
        # Store the resulting binary image in img_bin
        #
        img_gry = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_bin = cv2.Canny(img_gry, self.canny_lower, self.canny_upper)
        #
        #
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Canny", img_cor)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING HARRIS NODE - ", FULL_NAME)
        super().__init__("harris_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.callback_img, 1)
        self.declare_parameter("canny_l",10)
        self.declare_parameter("canny_u",20)
        self.canny_lower = self.get_parameter("canny_l").get_parameter_value().integer_value
        self.canny_upper = self.get_parameter("canny_u").get_parameter_value().integer_value
        print("Starting border detection with parameters: ", [self.canny_lower, self.canny_upper])
        
def main(args=None):
    rclpy.init(args=args)
    harris_node = HarrisNode()
    rclpy.spin(harris_node)
    harris_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
