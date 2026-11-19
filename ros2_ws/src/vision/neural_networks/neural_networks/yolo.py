import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import cv2
from sensor_msgs.msg import Image, JointState
#from cv_bridge import CvBridge
import numpy
import os
from ultralytics import YOLO

NAME = "FULL NAME"

class YoloNode(Node):
    def callback_img(self, msg):
        img_rgb = numpy.array(msg.data, dtype=numpy.uint8).reshape((480,640,3))
        r,g,b = cv2.split(img_rgb)
        img_bgr = cv2.merge([b,g,r])      
        #img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if img_bgr is None or not hasattr(img_bgr, 'shape'):
            self.get_logger().warn("Received empty or invalid image frame")
            return
        results = self.model(img_bgr, verbose=False)
        print(results[0])
        idxs = results[0].boxes.cls.cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()
        bboxes = results[0].boxes.xywh.cpu().tolist()
        print([idxs, confs, bboxes])
        annotated_frame = results[0].plot()
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)

    def __init__(self):
        print("INITIALIZING YOLO NODE - " + NAME)
        super().__init__("yolo_node")
        #self.br = CvBridge()
        model_path = os.path.join(get_package_share_directory("neural_networks"), "models", "yolov8n.pt")
        self.declare_parameter('model_path', model_path)
        #model_path  = self.get_parameter('model_path').get_parameter_value().string_value
        print(f'Initializing yolo model from path :{model_path}')
        self.model = YOLO(model_path)
        self.model.to('cuda')
        print("Model initialized succesfully")
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
