#
# MOBILE ROBOTS - FI-UNAM, 2027-1
# Object Recognition Node using YOLOv8
#

import os
import sys
import threading
import cv2
import numpy
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from pumas_vision_msgs.msg import VisionObject
from pumas_vision_msgs.srv import RecognizeObject, RecognizeObjects
from ament_index_python.packages import get_package_share_directory
from cv_bridge.core import CvBridge, CvBridgeError
from ultralytics import YOLO

NAME = "Medrano Solano Enrique / Orihuela Castillo Luis"

# Bidirectional alias map
CLASS_ALIASES = {
    "ball": ["ball", "sports ball", "pelota", "balon", "balón", "esfera"],
    "barbell": ["barbell", "dumbbell", "barra de pesas", "pesas", "pesa", "mancuerna", "mancuernas"],
    "bench_press": ["bench_press", "bench press", "banca", "banca de ejercicio", "banca de pesas", "aparato de ejercicio", "gimnasio"],
    "bed": ["bed", "cama", "colchon", "colchón"],
    "box": ["box", "caja", "empaque", "paquete", "carton", "cartón"],
    "fridge": ["fridge", "refrigerator", "refrigerador", "refri", "nevera"],
    "chair": ["chair", "silla", "asiento"],
    "photo": ["photo", "portrait", "cuadro", "retrato", "foto", "fotografia", "fotografía", "pintura"],
    "shoe": ["shoe", "tennis_shoe", "zapato", "zapatos", "tenis", "calzado"],
    "sofa": ["sofa", "couch", "sofá", "sillon", "sillón"],
    "table": ["table", "dining table", "mesa", "comedor", "escritorio"],
    "person": ["person", "persona", "humano", "gente"],
    "couch": ["couch", "sofa", "sofá", "sillon", "sillón"],
    "dining table": ["dining table", "table", "mesa", "comedor"],
    "tv": ["tv", "television", "televisión", "tele", "pantalla", "monitor"],
    "refrigerator": ["refrigerator", "fridge", "refrigerador", "refri", "nevera"],
    "vase": ["vase", "florero", "jarron", "jarrón"],
    "cup": ["cup", "mug", "taza", "vaso", "milk_cup", "coffe_mug"],
    "bottle": ["bottle", "botella", "envase"],
    "bowl": ["bowl", "tazon", "tazón", "plato"],
    "sports ball": ["sports ball", "ball", "pelota", "balon", "balón"],
    "potted plant": ["potted plant", "plant", "planta", "maceta"],
    "book": ["book", "libro"],
    "laptop": ["laptop", "computadora", "portatil", "portátil"],
    "clock": ["clock", "reloj"],
    "backpack": ["backpack", "mochila"],
    "coke_can": ["coke_can", "coca", "lata", "refresco", "coke"],
    "coke_green": ["coke_green", "lata verde", "refresco verde"],
    "coffe_mug": ["coffe_mug", "coffee_mug", "taza", "taza de cafe", "taza de café"],
    "milk_cup": ["milk_cup", "vaso de leche", "taza de leche", "vaso"],
    "mustard": ["mustard", "mostaza"],
    "cheezit": ["cheezit", "cheezit_big_original", "galletas", "caja de galletas"],
    "cheezit_big_original": ["cheezit_big_original", "cheezit", "galletas", "caja de galletas"],
    "nesquick": ["nesquick", "nesquik", "chocolate"],
    "snacks": ["snacks", "botana", "dulces", "gomitas"],
    "l_tyrosine": ["l_tyrosine", "suplemento", "frasco", "pastillas"],
    "u_by_kotex": ["u_by_kotex", "caja", "toallas"],
    "nescafe": ["nescafe", "nescafé", "cafe", "café"],
    "tennis_shoe": ["tennis_shoe", "tenis", "zapato"],
    "dumbbell": ["dumbbell", "mancuerna", "pesa"],
    "tablet": ["tablet", "tableta"],
}

EN_TO_ES = {
    "ball": "pelota",
    "barbell": "pesas",
    "bench_press": "banca de ejercicio",
    "bed": "cama",
    "box": "caja",
    "fridge": "refrigerador",
    "chair": "silla",
    "photo": "cuadro",
    "shoe": "zapato",
    "sofa": "sofá",
    "table": "mesa",
    "person": "persona",
    "couch": "sofá",
    "dining table": "mesa",
    "tv": "televisión",
    "refrigerator": "refrigerador",
    "vase": "florero",
    "cup": "taza",
    "bottle": "botella",
    "bowl": "tazón",
    "sports ball": "pelota",
    "potted plant": "planta",
    "book": "libro",
    "laptop": "computadora",
    "clock": "reloj",
    "backpack": "mochila",
    "coke_can": "lata de refresco",
    "coke_green": "lata verde",
    "coffe_mug": "taza",
    "milk_cup": "vaso de leche",
    "mustard": "mostaza",
    "cheezit": "caja de galletas",
    "cheezit_big_original": "caja de galletas",
    "nesquick": "bote de chocolate",
    "snacks": "bolsa de snacks",
    "l_tyrosine": "frasco de suplemento",
    "u_by_kotex": "caja",
    "nescafe": "caja de café",
    "tennis_shoe": "tenis",
    "dumbbell": "mancuerna",
    "tablet": "tableta",
}


class VenvCompatibleCvBridge(CvBridge):
    """Pure-NumPy/OpenCV CvBridge subclass avoiding cv_bridge_boost ABI errors under NumPy 2.x."""

    _ENCODING_MAP = {
        "mono8": ("uint8", 1),
        "8UC1": ("uint8", 1),
        "bgr8": ("uint8", 3),
        "rgb8": ("uint8", 3),
        "8UC3": ("uint8", 3),
        "bgra8": ("uint8", 4),
        "rgba8": ("uint8", 4),
        "8UC4": ("uint8", 4),
        "16UC1": ("uint16", 1),
        "mono16": ("uint16", 1),
        "32FC1": ("float32", 1),
    }

    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        """Convert sensor_msgs/Image to an OpenCV ndarray without C++ boost bindings."""
        enc = img_msg.encoding or "bgr8"
        if enc not in self._ENCODING_MAP:
            raise CvBridgeError(f"Unsupported image encoding: {enc}")

        dtype_name, n_channels = self._ENCODING_MAP[enc]
        dtype = numpy.dtype(dtype_name)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        img_buf = (
            numpy.asarray(img_msg.data, dtype=dtype)
            if isinstance(img_msg.data, list)
            else numpy.frombuffer(img_msg.data, dtype=dtype)
        )

        h = int(img_msg.height)
        w = int(img_msg.width)
        row_elems = int(img_msg.step // dtype.itemsize) if img_msg.step else w * n_channels

        if n_channels == 1:
            im = img_buf.reshape((h, row_elems))[:, :w]
        else:
            cols = max(w, row_elems // n_channels)
            im = img_buf.reshape((h, cols, n_channels))[:, :w, :]

        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        im = numpy.ascontiguousarray(im)

        if desired_encoding in ("passthrough", enc):
            return im

        if enc == "rgb8" and desired_encoding == "bgr8":
            return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        if enc == "bgr8" and desired_encoding == "rgb8":
            return cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        if enc == "rgba8" and desired_encoding == "bgr8":
            return cv2.cvtColor(im, cv2.COLOR_RGBA2BGR)
        if enc == "bgra8" and desired_encoding == "bgr8":
            return cv2.cvtColor(im, cv2.COLOR_BGRA2BGR)
        if enc in ("mono8", "8UC1") and desired_encoding == "bgr8":
            return cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        if enc == "bgr8" and desired_encoding in ("mono8", "8UC1"):
            return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        if enc == "rgb8" and desired_encoding in ("mono8", "8UC1"):
            return cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)

        raise CvBridgeError(f"Unsupported conversion from {enc} to {desired_encoding}")

    def cv2_to_imgmsg(self, cvim, encoding="passthrough", header=None):
        """Convert an OpenCV ndarray to sensor_msgs/Image without C++ boost bindings."""
        if not isinstance(cvim, (numpy.ndarray, numpy.generic)):
            raise TypeError("Input image is not a numpy ndarray")

        arr = numpy.ascontiguousarray(cvim)
        img_msg = Image()
        img_msg.height = int(arr.shape[0])
        img_msg.width = int(arr.shape[1])
        if header is not None:
            img_msg.header = header

        channels = 1 if arr.ndim == 2 else int(arr.shape[2])
        if encoding == "passthrough":
            if arr.dtype == numpy.uint8 and channels == 1:
                img_msg.encoding = "mono8"
            elif arr.dtype == numpy.uint8 and channels == 3:
                img_msg.encoding = "bgr8"
            elif arr.dtype == numpy.uint8 and channels == 4:
                img_msg.encoding = "bgra8"
            elif arr.dtype == numpy.uint16 and channels == 1:
                img_msg.encoding = "16UC1"
            elif arr.dtype == numpy.float32 and channels == 1:
                img_msg.encoding = "32FC1"
            else:
                raise CvBridgeError(f"Unsupported array dtype/channels: {arr.dtype}/{channels}")
        else:
            img_msg.encoding = encoding

        img_msg.is_bigendian = 1 if arr.dtype.byteorder == ">" else 0
        raw_bytes = arr.tobytes()
        img_msg.data = raw_bytes
        img_msg.step = len(raw_bytes) // max(1, img_msg.height)
        return img_msg


class ObjRecognizerNode(Node):
    """YOLOv8 object recognition node with PointCloud2 3D localization and annotated stream."""

    def __init__(self):
        super().__init__("obj_recognizer")
        self.get_logger().info("INITIALIZING OBJECT RECOGNIZER WITH YOLO - " + NAME)

        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.30)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("cloud_topic", "/camera/depth/points")
        self.declare_parameter("annotated_topic", "/vision/image_annotated")
        self.declare_parameter("show_window", False)

        param_model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.conf_threshold = self.get_parameter("conf_threshold").get_parameter_value().double_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        cloud_topic = self.get_parameter("cloud_topic").get_parameter_value().string_value
        annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.show_window = self.get_parameter("show_window").get_parameter_value().bool_value

        self.bridge = VenvCompatibleCvBridge()
        self._model_lock = threading.Lock()
        self.latest_img_msg = None
        self.latest_cloud_msg = None

        model_path = self._resolve_model_path(param_model_path)
        self.model_path = model_path
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLO model from '{model_path}' on device '{self.device}'...")
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # Warmup inference so service calls remain well below 1s timeouts
        warmup_img = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
        with self._model_lock:
            self.model.predict(warmup_img, conf=self.conf_threshold, verbose=False)
        self.get_logger().info("YOLO model initialized and warmed up successfully.")

        self.sub_img = self.create_subscription(Image, image_topic, self.callback_img, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, cloud_topic, self.callback_cloud, 10)
        self.pub_annotated = self.create_publisher(Image, annotated_topic, 10)

        self.srv_recog_obj = self.create_service(
            RecognizeObject, "/vision/recognize_object", self.callback_recognize_object
        )
        self.srv_recog_objs = self.create_service(
            RecognizeObjects, "/vision/recognize_objects", self.callback_recognize_objects
        )
        self.timer = self.create_timer(0.25, self.callback_timer)

    def _resolve_model_path(self, param_path):
        """Prefer custom trained .pt weights when present, falling back to yolov8s.pt."""
        if param_path and os.path.isfile(param_path):
            return param_path

        default_official = {"yolov8n.pt", "yolov8s.pt", "yolov8m.pt", "yolov8l.pt", "yolov8x.pt"}
        search_dirs = []
        try:
            search_dirs.append(os.path.join(get_package_share_directory("obj_recognizer"), "models"))
        except Exception:
            pass
        src_models = "/home/quique/Mobile-Robots-2027-1/ros2_ws/src/vision/obj_recognizer/models"
        if src_models not in search_dirs:
            search_dirs.append(src_models)

        for d in search_dirs:
            if not os.path.isdir(d):
                continue
            custom_candidate = os.path.join(d, "custom.pt")
            if os.path.isfile(custom_candidate):
                return custom_candidate
            for fname in sorted(os.listdir(d)):
                if fname.endswith(".pt") and fname not in default_official:
                    return os.path.join(d, fname)

        for d in search_dirs:
            official_path = os.path.join(d, "yolov8s.pt")
            if os.path.isfile(official_path):
                return official_path

        if os.path.isdir(src_models) and os.access(src_models, os.W_OK):
            return os.path.join(src_models, "yolov8s.pt")

        cache_dir = os.path.expanduser("~/.cache/yolo")
        os.makedirs(cache_dir, exist_ok=True)
        return os.path.join(cache_dir, "yolov8s.pt")

    def callback_img(self, msg):
        """Store latest camera RGB frame."""
        self.latest_img_msg = msg

    def callback_cloud(self, msg):
        """Store latest camera depth point cloud."""
        self.latest_cloud_msg = msg

    @staticmethod
    def _translate_to_spanish(label):
        """Map detected class label to natural Spanish name."""
        key = label.strip().lower()
        return EN_TO_ES.get(key, key.replace("_", " "))

    @staticmethod
    def _matches_target(detected_label, requested_id):
        """Check exact or bilingual alias match between detected label and requested object."""
        det = detected_label.strip().lower()
        req = requested_id.strip().lower()
        if not req:
            return False
        if det == req or req in det or det in req:
            return True
        aliases = CLASS_ALIASES.get(det, [det])
        for alias in aliases:
            if req == alias or req in alias:
                return True
        for base_label, group in CLASS_ALIASES.items():
            if req in group and det == base_label:
                return True
        return False

    def _extract_centroid_3d(self, xyz_array, x1, y1, x2, y2):
        """Compute median 3D centroid inside a 5x5 patch around bounding-box center."""
        if xyz_array is None:
            return None
        h, w, _ = xyz_array.shape
        cx = int(max(0, min(w - 1, (x1 + x2) // 2)))
        cy = int(max(0, min(h - 1, (y1 + y2) // 2)))
        y_min, y_max = max(0, cy - 2), min(h, cy + 3)
        x_min, x_max = max(0, cx - 2), min(w, cx + 3)
        patch = xyz_array[y_min:y_max, x_min:x_max].reshape(-1, 3)
        valid = numpy.all(numpy.isfinite(patch), axis=1)
        if not numpy.any(valid):
            return None
        med = numpy.median(patch[valid], axis=0)
        return float(med[0]), float(med[1]), float(med[2])

    def _run_detection(self, img_msg, cloud_msg=None):
        """Run YOLOv8 inference on ROS Image and optionally attach 3D coordinates."""
        img_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        xyz_array = None
        if cloud_msg is not None and cloud_msg.width > 0 and cloud_msg.height > 0:
            try:
                pts = read_points_numpy(cloud_msg, field_names=["x", "y", "z"], skip_nans=False)
                xyz_array = pts.reshape((cloud_msg.height, cloud_msg.width, 3))
            except Exception as exc:
                self.get_logger().warn(f"Could not parse PointCloud2: {exc}")

        with self._model_lock:
            results = self.model.predict(img_bgr, conf=self.conf_threshold, verbose=False)

        vision_objs = []
        annotated = img_bgr.copy()
        if results:
            res0 = results[0]
            names = res0.names
            for box in res0.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = str(names.get(cls_id, cls_id))
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

                vobj = VisionObject()
                vobj.header = img_msg.header
                vobj.id = label
                vobj.category = self._translate_to_spanish(label)
                vobj.confidence = conf
                vobj.x = x1
                vobj.y = y1
                vobj.width = max(0, x2 - x1)
                vobj.height = max(0, y2 - y1)

                centroid = self._extract_centroid_3d(xyz_array, x1, y1, x2, y2)
                if centroid is not None:
                    vobj.pose.position.x, vobj.pose.position.y, vobj.pose.position.z = centroid
                vobj.pose.orientation.w = 1.0
                vision_objs.append(vobj)

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 220, 0), 2)
                tag = f"{vobj.category} ({conf:.2f})"
                cv2.putText(
                    annotated,
                    tag,
                    (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 220, 0),
                    2,
                )

        hud_text = f"YOLO ({os.path.basename(self.model_path)}) | Objetos: {len(vision_objs)}"
        cv2.putText(
            annotated,
            hud_text,
            (10, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        self.pub_annotated.publish(
            self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8", header=img_msg.header)
        )
        return vision_objs, annotated

    def callback_timer(self):
        """Publish annotated detections periodically and update optional preview window."""
        if self.latest_img_msg is None:
            return
        _, annotated = self._run_detection(self.latest_img_msg, None)
        if self.show_window:
            cv2.imshow("YOLOv8 Detections", annotated)
            cv2.waitKey(1)

    def callback_recognize_objects(self, req, res):
        """Service handler returning all objects currently detected by YOLOv8."""
        img_msg = req.img_bgr if req.img_bgr.width > 0 and req.img_bgr.height > 0 else self.latest_img_msg
        cloud_msg = req.cloud if req.cloud.width > 0 and req.cloud.height > 0 else self.latest_cloud_msg
        if img_msg is None:
            self.get_logger().warn("No camera image available yet for /vision/recognize_objects")
            return res
        objs, _ = self._run_detection(img_msg, cloud_msg)
        res.objs = objs
        self.get_logger().info(f"Recognized {len(objs)} object(s).")
        return res

    def callback_recognize_object(self, req, res):
        """Service handler searching for a specific object ID or Spanish/English alias."""
        self.get_logger().info(f"Trying to recognize '{req.id}'")
        img_msg = req.img_bgr if req.img_bgr.width > 0 and req.img_bgr.height > 0 else self.latest_img_msg
        cloud_msg = req.cloud if req.cloud.width > 0 and req.cloud.height > 0 else self.latest_cloud_msg
        if img_msg is None:
            self.get_logger().warn("No camera image available yet for /vision/recognize_object")
            return res

        objs, _ = self._run_detection(img_msg, cloud_msg)
        matches = [o for o in objs if self._matches_target(o.id, req.id)]
        if matches:
            best = max(matches, key=lambda o: o.confidence)
            # Match lira_gui expectation (result->obj.id == req.id) while keeping Spanish category
            best.id = req.id
            res.obj = best
            self.get_logger().info(
                f"Found '{req.id}' ({best.category}) conf={best.confidence:.2f}"
            )
        else:
            self.get_logger().info(f"Object '{req.id}' not found in current frame.")
        return res


def main(args=None):
    rclpy.init(args=args)
    n = ObjRecognizerNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
