import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from pumas_vision_msgs.srv import RecognizeObject
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class ObjRecognizerNode(Node):
    def __init__(self):
        super().__init__("obj_recognizer")

        model_path = os.path.join(
            get_package_share_directory("obj_recognizer"),
            "models", "yolov8n.pt"
        )
        self.declare_parameter("model_path", model_path)
        self.declare_parameter("image_topic", "/camera/color/image_raw")

        self.model = YOLO(
            self.get_parameter("model_path").value
        )
        self.model.to("cpu")

        self.last_image = None
        self.last_image_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.receive_image,
            qos_profile_sensor_data
        )
        self.image_publisher = self.create_publisher(
            Image, "/vision/recognized_image", 10
        )
        self.service = self.create_service(
            RecognizeObject,
            "/vision/recognize_object",
            self.recognize
        )

        self.get_logger().info(
            "YOLO listo en CPU. Esperando imagenes y solicitudes."
        )

    def receive_image(self, msg):
        self.last_image = msg
        self.last_image_time = time.monotonic()

    def to_bgr(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(
                f"Formato de imagen no soportado: {msg.encoding}"
            )

        rows = np.frombuffer(
            bytes(msg.data), dtype=np.uint8
        ).reshape(msg.height, msg.step)

        image = rows[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3
        ).copy()

        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        return image

    def to_message(self, image, header):
        image = np.ascontiguousarray(image)
        msg = Image()
        msg.header = header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = image.tobytes()
        return msg

    def recognize(self, request, response):
        # La solicitud puede traer una imagen.
        # Si no la trae, se usa la ultima imagen de la camara.
        if request.img_bgr.height > 0:
            msg = request.img_bgr
        else:
            msg = self.last_image
            if msg is None or time.monotonic() - self.last_image_time > 20:
                self.get_logger().warning(
                    "No hay una imagen reciente de la camara."
                )
                return response

        try:
            image = self.to_bgr(msg)
            result = self.model.predict(
                source=image,
                device="cpu",
                conf=0.35,
                verbose=False
            )[0]

            self.image_publisher.publish(
                self.to_message(result.plot(), msg.header)
            )

            # Algunas traducciones para las ordenes en espanol.
            aliases = {
                "botella": "bottle",
                "taza": "cup",
                "silla": "chair",
                "persona": "person",
                "libro": "book",
                "celular": "cell phone"
            }
            target = request.id.strip().lower()
            target = aliases.get(target, target)

            candidates = []
            for box in result.boxes:
                name = result.names[int(box.cls.item())]
                if not target or name.lower() == target:
                    candidates.append((float(box.conf.item()), name, box))

            if not candidates:
                self.get_logger().info(
                    f"No se detecto el objeto solicitado: {target or 'cualquiera'}"
                )
                return response

            confidence, name, box = max(
                candidates, key=lambda item: item[0]
            )
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            height, width = image.shape[:2]
            x1 = max(0, min(width - 1, int(x1)))
            y1 = max(0, min(height - 1, int(y1)))
            x2 = max(x1 + 1, min(width, int(np.ceil(x2))))
            y2 = max(y1 + 1, min(height, int(np.ceil(y2))))

            obj = response.obj
            obj.header = msg.header
            obj.id = name
            obj.confidence = confidence
            obj.x, obj.y = x1, y1
            obj.width, obj.height = x2 - x1, y2 - y1
            obj.image = self.to_message(
                image[y1:y2, x1:x2], msg.header
            )

            # Todavia no se calcula posicion 3D.
            obj.pose.position.x = float("nan")
            obj.pose.position.y = float("nan")
            obj.pose.position.z = float("nan")
            obj.graspable = False

            self.get_logger().info(
                f"Objeto detectado: {name}, confianza: {confidence:.2f}"
            )

        except Exception as error:
            self.get_logger().error(
                f"Error al reconocer el objeto: {error}"
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()