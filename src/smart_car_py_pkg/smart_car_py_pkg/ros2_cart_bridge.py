import json
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


ROOT = Path(__file__).resolve().parent


def resolve_model_path() -> Path:
    candidates = []
    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory("smart_car_py_pkg"))
            candidates.extend([
                share_dir / "models" / "best.pt",
                share_dir / "models" / "yolov8n.pt",
            ])
        except Exception:
            pass

    candidates.extend([
        ROOT.parent / "pc" / "best.pt",
        ROOT.parent / "pc" / "yolov8n.pt",
        ROOT / "models" / "best.pt",
        ROOT / "best.pt",
        ROOT / "best_old.pt",
        ROOT / "yolov8n.pt",
    ])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def normalize_class_name(class_name: str) -> str:
    return str(class_name or "").strip().lower().replace(" ", "_").replace("-", "_")


class Ros2CartBridge(Node):
    def __init__(self):
        super().__init__("ros2_cart_bridge")

        self.declare_parameter("image_topic", "/webcam/image_raw/compressed")
        self.declare_parameter("model_path", str(resolve_model_path()))
        self.declare_parameter("confidence", 0.6)
        self.declare_parameter("cooldown", 3.0)
        self.declare_parameter("cart_api_url", "http://127.0.0.1:5000/api/add_item")

        self.image_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence = float(self.get_parameter("confidence").value)
        self.cooldown = float(self.get_parameter("cooldown").value)
        self.cart_api_url = self.get_parameter("cart_api_url").value

        self.model = YOLO(self.model_path)
        self.last_sent = {}

        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info(f"Subscribing compressed image topic: {self.image_topic}")
        self.get_logger().info(f"YOLO model: {self.model_path}")
        self.get_logger().info(f"Cart API: {self.cart_api_url}")

    def image_callback(self, msg: CompressedImage):
        frame = self._decode_compressed_image(msg)
        if frame is None:
            return

        results = self.model(frame, verbose=False, conf=self.confidence)
        now = time.time()

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = normalize_class_name(self.model.names[class_id])

                if now - self.last_sent.get(class_name, 0) < self.cooldown:
                    continue

                if self._send_to_cart(class_name):
                    self.last_sent[class_name] = now
                    self.get_logger().info(f"sent to cart: {class_name} confidence={confidence:.2f}")

    def _decode_compressed_image(self, msg: CompressedImage):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image frame")
        return frame

    def _send_to_cart(self, class_name: str) -> bool:
        payload = json.dumps({"class_name": class_name}).encode("utf-8")
        request = urllib.request.Request(
            self.cart_api_url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                return 200 <= response.status < 300
        except Exception as exc:
            self.get_logger().warning(f"Cart API request failed: {exc}")
            return False


def main():
    rclpy.init(args=None)
    node = Ros2CartBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
