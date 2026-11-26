"""Recognize objects in the simulated camera with YOLO and color.

The service keeps the latest image from the camera and answers
/vision/recognize_object and /vision/recognize_objects.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, PointCloud2
from pumas_vision_msgs.msg import VisionObject
from pumas_vision_msgs.srv import RecognizeObject, RecognizeObjects

try:
    import cv2
    import numpy
except ImportError:
    cv2 = None
    numpy = None


COCO_ES = {
    'person': 'persona',
    'chair': 'silla',
    'couch': 'sofa',
    'bed': 'cama',
    'dining table': 'mesa',
    'tv': 'television',
    'bottle': 'botella',
    'cup': 'taza',
    'book': 'libro',
    'laptop': 'laptop',
    'keyboard': 'teclado',
    'mouse': 'mouse',
    'remote': 'control',
    'cell phone': 'telefono',
    'microwave': 'microondas',
    'oven': 'horno',
    'refrigerator': 'refrigerador',
    'sink': 'fregadero',
    'vase': 'florero',
    'potted plant': 'planta',
    'clock': 'reloj',
    'teddy bear': 'peluche',
    'backpack': 'mochila',
    'handbag': 'bolsa',
    'suitcase': 'maleta',
    'sports ball': 'pelota',
}

# OpenCV hue is 0-180. Two ranges cover red, which wraps around 0.
_COLORS = (
    ('coca', (
        ((0, 110, 70), (10, 255, 255)),
        ((170, 110, 70), (180, 255, 255)),
    )),
    ('mostaza', (
        ((18, 110, 90), (36, 255, 255)),
    )),
    ('cheezit', (
        ((8, 130, 110), (18, 255, 255)),
    )),
)


class ObjRecognizerSolved(Node):
    """YOLO plus color blobs for the objects in the house."""

    def __init__(self):
        super().__init__('obj_recognizer_solved')
        self.last_bgr = None
        self.last_cloud = None
        self.model = self._load_model()
        image_topics = (
            '/camera/color/image_raw',
            '/camera/color/image_optical',
            '/camera/image_raw',
        )
        for topic in image_topics:
            self.create_subscription(
                Image, topic, self._on_image, qos_profile_sensor_data)
        cloud_topics = (
            '/camera/depth/points',
            '/camera/depth/points_optical',
            '/camera/depth/points_depth',
        )
        for topic in cloud_topics:
            self.create_subscription(
                PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 1)
        self.srv_one = self.create_service(
            RecognizeObject, '/vision/recognize_object', self._on_one)
        self.srv_all = self.create_service(
            RecognizeObjects, '/vision/recognize_objects', self._on_all)
        self.get_logger().info('Reconocedor listo')

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().warning(
                'ultralytics no está instalado. Solo usaré color.')
            return None
        share = get_package_share_directory('obj_recognizer')
        packaged = os.path.join(share, 'models', 'yolov8n.pt')
        source = packaged if os.path.isfile(packaged) else 'yolov8n.pt'
        try:
            model = YOLO(source)
        except Exception as exc:
            self.get_logger().warning('No pude cargar YOLO: %s' % exc)
            return None
        try:
            model.to('cuda')
            self.get_logger().info('YOLO en CUDA')
        except Exception:
            model.to('cpu')
            self.get_logger().info('YOLO en CPU')
        return model

    def _on_image(self, msg):
        image = self._to_bgr(msg)
        if image is not None:
            self.last_bgr = image

    def _on_cloud(self, msg):
        self.last_cloud = msg

    def _to_bgr(self, msg):
        if numpy is None or cv2 is None or msg.height == 0 or msg.width == 0:
            return None
        encoding = (msg.encoding or '').lower()
        array = numpy.frombuffer(msg.data, dtype=numpy.uint8)
        if encoding in ('rgb8', 'bgr8') and array.size == msg.height * msg.width * 3:
            image = array.reshape((msg.height, msg.width, 3))
            if encoding == 'rgb8':
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image.copy()
        if encoding in ('rgba8', 'bgra8') and array.size == msg.height * msg.width * 4:
            image = array.reshape((msg.height, msg.width, 4))
            code = cv2.COLOR_RGBA2BGR if encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(image, code)
        if array.size == msg.height * msg.width * 3:
            image = array.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return None

    def _on_one(self, request, response):
        found = self._detect(request.id)
        if found:
            response.obj = found[0]
            self.get_logger().info('Encontré %s' % response.obj.id)
        else:
            self.get_logger().info('No encontré %s' % request.id)
        return response

    def _on_all(self, request, response):
        del request
        response.objs = self._detect(None)
        names = ', '.join(item.id for item in response.objs) or 'ninguno'
        self.get_logger().info('Objetos: ' + names)
        return response

    def _detect(self, requested):
        if self.last_bgr is None or cv2 is None:
            self.get_logger().warning('No hay imagen de la cámara')
            return []
        image = self.last_bgr.copy()
        found = []
        found.extend(self._yolo(image))
        found.extend(self._colors(image))
        if requested and requested not in ('', 'all', 'any', 'algo'):
            found = [item for item in found if self._same(item['id'], requested)]
        objects = []
        seen = set()
        for item in found:
            if item['id'] in seen:
                continue
            seen.add(item['id'])
            objects.append(self._to_msg(item))
            cv2.rectangle(
                image,
                (item['x'], item['y']),
                (item['x'] + item['w'], item['y'] + item['h']),
                (0, 220, 0),
                2,
            )
        self.debug_pub.publish(self._to_image(image))
        return objects

    def _yolo(self, image):
        if self.model is None:
            return []
        try:
            results = self.model.predict(image, verbose=False, conf=0.25)
        except Exception as exc:
            self.get_logger().warning('YOLO falló: %s' % exc)
            return []
        found = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0])
                names = self.model.names
                if isinstance(names, dict):
                    english = names.get(cls_id, str(cls_id))
                else:
                    english = names[cls_id]
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                found.append({
                    'id': COCO_ES.get(english, english),
                    'category': english,
                    'confidence': float(box.conf[0]),
                    'x': max(0, x1),
                    'y': max(0, y1),
                    'w': max(1, x2 - x1),
                    'h': max(1, y2 - y1),
                })
        return found

    def _colors(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        found = []
        kernel = numpy.ones((5, 5), numpy.uint8)
        for name, ranges in _COLORS:
            mask = numpy.zeros((height, width), numpy.uint8)
            for low, high in ranges:
                part = cv2.inRange(hsv, numpy.array(low), numpy.array(high))
                mask = cv2.bitwise_or(mask, part)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _hier = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best = None
            best_area = 400
            for contour in contours:
                area = cv2.contourArea(contour)
                x_pix, y_pix, box_w, box_h = cv2.boundingRect(contour)
                if box_w > 0.85 * width and box_h > 0.85 * height:
                    continue
                if area > best_area:
                    best_area = area
                    best = (x_pix, y_pix, box_w, box_h, area)
            if best is None:
                continue
            x_pix, y_pix, box_w, box_h, area = best
            found.append({
                'id': name,
                'category': 'color',
                'confidence': min(0.95, 0.45 + area / 20000.0),
                'x': int(x_pix),
                'y': int(y_pix),
                'w': int(box_w),
                'h': int(box_h),
            })
        return found

    def _same(self, detected, requested):
        left = detected.lower().strip()
        right = requested.lower().strip()
        return left == right or left in right or right in left

    def _to_msg(self, item):
        msg = VisionObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.id = item['id']
        msg.category = item['category']
        msg.confidence = float(item['confidence'])
        msg.x = int(item['x'])
        msg.y = int(item['y'])
        msg.width = int(item['w'])
        msg.height = int(item['h'])
        msg.graspable = item['category'] == 'color'
        point = self._point_at(
            item['x'] + item['w'] // 2, item['y'] + item['h'] // 2)
        msg.pose = Pose()
        if point is not None:
            msg.pose.position.x = point[0]
            msg.pose.position.y = point[1]
            msg.pose.position.z = point[2]
        msg.pose.orientation.w = 1.0
        return msg

    def _point_at(self, pixel_x, pixel_y):
        cloud = self.last_cloud
        if cloud is None or numpy is None or cloud.height < 2:
            return None
        try:
            from sensor_msgs_py import point_cloud2
            points = point_cloud2.read_points_numpy(
                cloud, field_names=['x', 'y', 'z'], skip_nans=False)
            points = points.reshape((cloud.height, cloud.width, 3))
        except Exception:
            return None
        if not (0 <= pixel_y < points.shape[0] and 0 <= pixel_x < points.shape[1]):
            return None
        x_m, y_m, z_m = points[int(pixel_y), int(pixel_x)]
        if not numpy.isfinite([x_m, y_m, z_m]).all():
            return None
        return float(x_m), float(y_m), float(z_m)

    def _to_image(self, image):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = numpy.ascontiguousarray(image).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ObjRecognizerSolved()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
