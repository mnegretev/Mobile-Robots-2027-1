#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# CANNY EDGE DETECTOR
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2
from rclpy.qos import qos_profile_sensor_data

FULL_NAME = "LEONARDO ALEJANDRO GARCIA LERMA"

class CannyNode(Node):
    def callback_img(self, msg):
        print("¡Imagen recibida en el callback!")
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 1. Convertir a escala de grises
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        
        # 2. Aplicar filtro gaussiano
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 3. Detectar bordes con Canny
        img_bin = cv2.Canny(blur, self.canny_lower, self.canny_upper)
        
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Canny", img_bin)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING CANNY NODE - ", FULL_NAME)
        super().__init__("canny_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, qos_profile_sensor_data)
        self.declare_parameter("canny_l",10)
        self.declare_parameter("canny_u",20)
        self.canny_lower = self.get_parameter("canny_l").get_parameter_value().integer_value
        self.canny_upper = self.get_parameter("canny_u").get_parameter_value().integer_value
        print("Starting border detection with parameters: ", [self.canny_lower, self.canny_upper])
        print("Esperando recibir imágenes del tópico /camera/color/image_raw...")
        

def main(args=None):
    rclpy.init(args=args)
    canny_node = CannyNode()
    rclpy.spin(canny_node)
    canny_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()