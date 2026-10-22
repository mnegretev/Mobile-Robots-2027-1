#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# CANNY EDGE DETECTOR
#
# Instructions:
# Complete the code necessary to implement the Canny edge detector
# using the function provided by OpenCV
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2

FULL_NAME = "Omar Rosario Carrasco"

class CannyNode(Node):
    def callback_img(self, msg):
        img_bin = numpy.zeros((480, 640))
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #
        # TODO:
        # Change the color space of the image 'img_bgr' to grayscale
        # Get edges using the cv2.Canny function, use as parameters
        # the variables self.canny_lower and self.canny_upper
        # Store the resulting binary image in img_bin
        #
        # 1) BGR -> escala de grises: los bordes se buscan en la intensidad (1 canal)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        # 2) Filtro gaussiano 5x5 (sigma lo calcula OpenCV a partir del kernel).
        #    cv2.Canny NO suaviza por sí mismo: sin este paso el ruido y la textura
        #    del piso generan bordes espurios.
        img_gray = cv2.GaussianBlur(img_gray, (5, 5), 0)
        # 3) Canny: gradiente con Sobel, supresión de no-máximos e histéresis.
        #    Magnitud > canny_upper -> borde fuerte; entre canny_lower y canny_upper
        #    -> borde solo si está conectado a un borde fuerte; < canny_lower -> descartado.
        img_bin = cv2.Canny(img_gray, self.canny_lower, self.canny_upper)
        #
        #
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Canny", img_bin)
        cv2.waitKey(1)
    
    def __init__(self):
        print("INITIALIZING CANNY NODE - ", FULL_NAME)
        super().__init__("canny_node")
        self.br = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.callback_img, 1)
        # En house_simul (realsense:=True) el bridge de Gazebo publica la imagen RGB en
        # /camera/color/image_raw (ver mbot_demo/config/gz_bridge.yaml), no en /camera/image_raw.
        # Se escuchan ambos tópicos para que el nodo reciba imágenes en cualquiera de los dos casos.
        self.sub_img_color = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
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
