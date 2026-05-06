import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import threading
from ultralytics import YOLO


class YoloCompressedViewer(Node):
    def __init__(self):
        super().__init__('yolo_compressed_viewer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.lock = threading.Lock()
        self.latest_bytes = None
        self.frame_ok = 0
        self.frame_drop = 0
        self.infer_count = 0

        self.model = YOLO('yolov8n.pt')

        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            qos
        )

        # 너무 빠르면 YOLO가 못 따라갈 수 있어서 30Hz 대신 약 15~20Hz 권장
        self.timer = self.create_timer(0.05, self.process_frame)

    def callback(self, msg):
        # 최신 프레임만 유지
        with self.lock:
            self.latest_bytes = bytes(msg.data)

    def process_frame(self):
        with self.lock:
            if self.latest_bytes is None:
                return
            data = self.latest_bytes
            self.latest_bytes = None   # 오래된 프레임 버리기

        arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.frame_drop += 1
            return

        self.frame_ok += 1

        # YOLO 속도 때문에 해상도 줄이기
        frame = cv2.resize(frame, (320, 240))

        # YOLO 추론
        results = self.model(frame, verbose=False)
        annotated = results[0].plot()
        self.infer_count += 1

        # 상태 표시
        status_text = f"recv:{self.frame_ok} drop:{self.frame_drop} infer:{self.infer_count}"
        cv2.putText(
            annotated,
            status_text,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA
        )

        cv2.imshow("TurtleBot3 YOLO Detection", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:   # ESC
            rclpy.shutdown()


def main():
    rclpy.init()
    node = YoloCompressedViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
