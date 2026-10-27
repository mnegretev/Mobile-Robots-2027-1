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
import json, os   # van después de FULL_NAME para que el nombre siga en la línea 17

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
        # 2) Filtro gaussiano de blur x blur píxeles (parámetro 'blur', 5 por defecto;
        #    sigma lo calcula OpenCV a partir del kernel). cv2.Canny NO suaviza por sí
        #    mismo: sin este paso el ruido y la textura del piso generan bordes espurios.
        if self.blur > 1:
            img_gray = cv2.GaussianBlur(img_gray, (self.blur, self.blur), 0)
        # 3) Canny: gradiente con Sobel, supresión de no-máximos e histéresis.
        #    Magnitud > canny_upper -> borde fuerte; entre canny_lower y canny_upper
        #    -> borde solo si está conectado a un borde fuerte; < canny_lower -> descartado.
        img_bin = cv2.Canny(img_gray, self.canny_lower, self.canny_upper)
        #
        #
        self.frame_count += 1
        if self.log_file is not None:
            self.log_frame(img_bin, img_bgr)
        cv2.imshow("BGR Original", img_bgr)
        cv2.imshow("Canny", img_bin)
        cv2.waitKey(1)

    def log_frame(self, img_bin, img_bgr):
        # Modo experimento: píxeles de borde de cada cuadro; al llegar a n_frames guarda
        # las imágenes, los parámetros usados y el archivo 'done' que espera el script.
        self.log_file.write("%d,%d\n" % (self.frame_count, cv2.countNonZero(img_bin)))
        if self.frame_count < self.n_frames:
            return
        self.log_file.close()
        self.log_file = None
        cv2.imwrite(os.path.join(self.log_dir, "original.png"), img_bgr)
        cv2.imwrite(os.path.join(self.log_dir, "bordes.png"), img_bin)
        names = ["canny_l", "canny_u", "blur", "n_frames", "use_sim_time"]
        with open(os.path.join(self.log_dir, "params.json"), "w") as f:
            json.dump({name: self.get_parameter(name).value for name in names}, f, indent=1)
        open(os.path.join(self.log_dir, "done"), "w").close()
        self.get_logger().info("Experimento terminado: %d cuadros en %s" % (self.n_frames, self.log_dir))

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
        self.declare_parameter("blur", 5)        # kernel del filtro gaussiano (impar); 0 = sin filtro
        self.declare_parameter("log_dir", "")    # modo experimento: carpeta de salida ("" = desactivado)
        self.declare_parameter("n_frames", 20)   # modo experimento: cuadros a registrar
        self.canny_lower = self.get_parameter("canny_l").get_parameter_value().integer_value
        self.canny_upper = self.get_parameter("canny_u").get_parameter_value().integer_value
        self.blur = self.get_parameter("blur").get_parameter_value().integer_value
        if self.blur > 1 and self.blur % 2 == 0:
            self.blur += 1                       # GaussianBlur solo acepta kernels impares
        self.frame_count = 0
        self.log_dir  = self.get_parameter("log_dir").get_parameter_value().string_value
        self.n_frames = self.get_parameter("n_frames").get_parameter_value().integer_value
        self.log_file = None
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            self.log_file = open(os.path.join(self.log_dir, "frames.csv"), "w")
            self.log_file.write("frame,edge_px\n")
        print("Starting border detection with parameters: ", [self.canny_lower, self.canny_upper])
        print("Gaussian blur kernel: ", self.blur if self.blur > 1 else "none")
        if self.log_dir:
            print("Experiment mode: %d frames -> %s" % (self.n_frames, self.log_dir))


def main(args=None):
    rclpy.init(args=args)
    canny_node = CannyNode()
    rclpy.spin(canny_node)
    canny_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
