#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# APRIL TAG DETECTION USING OPENCV
#
# Instructions:
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy
import cv2
import apriltag 

NAME = "FULL NAME"

class AprilTagNode(Node):
    def callback_img(self, msg):
        img_rgb = numpy.array(msg.data, dtype=numpy.uint8).reshape((480,640,3))
        r,g,b = cv2.split(img_rgb)
        img_bgr = cv2.merge([b,g,r])      
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        

        cv2.imshow("Detections", img_bgr)
        cv2.waitKey(10)

    def __init__(self):
        super().__init__("april_tag_node")
        self.get_logger().info("INITIALIZING APRIL TAG READER NODE - " + NAME)
        self.sub_img = self.create_subscription(Image, "/camera/color/image_raw", self.callback_img, 1)

def main(args=None):
        rclpy.init(args=args)
        n = AprilTagNode()
        rclpy.spin(n)
        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
        
        
