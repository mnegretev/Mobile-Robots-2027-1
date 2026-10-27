import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2

FULL_NAME = "Abraham Pérez Rojo"

class CannyNode(Node):
    def callback_img(self, msg):
        img_bin = numpy.zeros((480, 640), dtype=numpy.uint8)
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Conversión a escala de grises, filtro Gaussiano y Canny
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        img_bin = cv2.Canny(img_blur, self.canny_lower, self.canny_upper)
        
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Canny", img_bin)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING CANNY NODE - ", FULL_NAME)
        super().__init__("canny_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        self.declare_parameter("canny_l",10)
        self.declare_parameter("canny_u",20)
        self.canny_lower = self.get_parameter("canny_l").get_parameter_value().integer_value
        self.canny_upper = self.get_parameter("canny_u").get_parameter_value().integer_value
        print("Starting border detection with parameters: ", [self.canny_lower, self.canny_upper])
        
def main(args=None):
    rclpy.init(args=args)
    canny_node = CannyNode()
    rclpy.spin(canny_node)
    canny_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()