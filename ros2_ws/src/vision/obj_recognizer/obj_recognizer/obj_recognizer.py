import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image, PointCloud2, JointState
from sensor_msgs_py.point_cloud2 import read_points_numpy
from pumas_vision_msgs.srv import RecognizeObject, RecognizeObjects
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy
import os
from ultralytics import YOLO

NAME = "FULL NAME"

class ObjRecognizerNode(Node):
    def recognize_object(self, img_bgr, img_xyz, obj_name):
        print('Testing1')
        img_x, img_y, cloud_x, cloud_y, cloud_z = 0,0,0,0,0

    def callback_cloud(self, msg):
        xyz_array = read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=False)
        xyz_array = xyz_array.reshape((msg.height, msg.width, 3))
        self.cloud = xyz_array

    def callback_recognize_object(self, req, res):
        self.get_logger().info(f'Trying to recognize {req.id}')
        self.get_logger().info("Waiting for image and point cloud...")
        success_img,   msg_img   = wait_for_message(Image, self, '/camera/color/image_raw', time_to_wait=0.5)
        success_cloud, msg_cloud = wait_for_message(PointCloud2, self, '/camera/depth/points', time_to_wait=0.5)
        if not success_img or not success_cloud:
            self.get_logger().info("Cannot get topics from camera")
            return res
        img_rgb = numpy.array(msg_img.data, dtype=numpy.uint8).reshape((480,640,3))
        r,g,b = cv2.split(img_rgb)
        img_bgr = cv2.merge([b,g,r])
        xyz_array = read_points_numpy(msg_cloud, field_names=['x', 'y', 'z'], skip_nans=False)
        xyz_array = xyz_array.reshape((msg_cloud.height, msg_cloud.width, 3))
        self.recognize_object(img_bgr, xyz_array, req.id)
        return res

    def callback_recognize_objects(self, req, res):
        #response.sum = request.a + request.b
        self.get_logger().info(f'Trying to recognize objects')
        return res

    def __init__(self):
        super().__init__("obj_recognizer")
        self.get_logger().info("INITIALIZING OBJECT RECOGNIZER WITH YOLO - " + NAME)
        model_path = os.path.join(get_package_share_directory("obj_recognizer"), "models", "yolov8n.pt")
        self.declare_parameter('model_path', model_path)
        self.get_logger().info(f'Initializing yolo model from path :{model_path}')
        self.model = YOLO(model_path)
        self.model.to('cuda')
        self.get_logger().info("Model initialized succesfully")
        self.srv_recog_obj  = self.create_service(RecognizeObject,  '/vision/recognize_object',  self.callback_recognize_object)
        self.srv_recog_objs = self.create_service(RecognizeObjects, '/vision/recognize_objects', self.callback_recognize_objects)
        #self.cloud = None
        #self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        #self.sub_xyz = self.create_subscription(PointCloud2, '/camera/depth/points', self.callback_cloud, 1)

def main(args=None):
    rclpy.init(args=args)
    n = ObjRecognizerNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
