import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from ultralytics import YOLO


class SmartCartTracker(Node):
    def __init__(self):
        super().__init__('smart_cart_tracker_pc')
        self.bridge = CvBridge()
        self.declare_parameter('process_period_sec', 0.08)
        self.declare_parameter('yolo_imgsz', 320)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/webcam2/image_raw/compressed',
            self.image_callback,
            qos,
        )
        self.servo_pan_pub = self.create_publisher(Int32, '/servo_pan_cmd', 10)
        # self.servo_tilt_pub = self.create_publisher(Int32, '/servo_tilt_cmd', 10)
        self.pan_angle_pub = self.create_publisher(
            Float64, '/pan_tilt/pan_angle', 10)
        self.detection_pub = self.create_publisher(
            Float32MultiArray, '/person_detection', 10)

        self.model = YOLO(self._resolve_model_path('yolov8n.pt'))
        self.master_db = []
        self.is_learning = True
        self.start_time = time.time()
        self.learning_duration = 30
        self.initial_pan_us = 1500
        self.current_pan_angle = self._microseconds_to_angle(self.initial_pan_us)
        self.target_pan_angle = self.current_pan_angle

        self.last_status_log = 0.0
        self.frame_ok = 0
        self.infer_count = 0
        self.latest_image_msg = None
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.initial_pose_timer = self.create_timer(
            1.0, self.publish_initial_pose)
        self.process_timer = self.create_timer(
            float(self.get_parameter('process_period_sec').value),
            self.process_latest_frame)

        self.get_logger().info(
            'PC tracker node started. Waiting for Raspberry Pi camera images...'
        )

    def _microseconds_to_angle(self, microseconds):
        return float(np.interp(microseconds, [500, 2500], [0, 180]))

    def _servo_angle_to_centered_rad(self, angle):
        return float(np.deg2rad(angle - 90.0))

    def publish_initial_pose(self):
        self.servo_pan_pub.publish(Int32(data=self.initial_pan_us))
        self.publish_pan_angle()
        self.initial_pose_timer.cancel()
        self.get_logger().info(
            f'Initial pan pose published: pan={self.initial_pan_us}us')

    def publish_pan_angle(self):
        self.pan_angle_pub.publish(
            Float64(data=self._servo_angle_to_centered_rad(self.current_pan_angle)))

    def publish_person_detection(self, frame, x1, x2, confidence):
        msg = Float32MultiArray()
        msg.data = [
            float((x1 + x2) / 2.0),
            float(frame.shape[1]),
            float(confidence),
        ]
        self.detection_pub.publish(msg)

    def _resolve_model_path(self, model_name):
        candidates = []
        try:
            share_dir = get_package_share_directory('smart_car_py_pkg')
            candidates.append(os.path.join(share_dir, 'models', model_name))
        except Exception:
            pass

        candidates.extend([
            os.path.join(os.getcwd(), model_name),
            os.path.join(os.getcwd(), 'src', 'smart_car_py_pkg', 'pc', model_name),
            os.path.expanduser(
                os.path.join('~/smartcar_ws/src/smart_car_py_pkg/pc', model_name)
            ),
        ])

        for path in candidates:
            if path and os.path.exists(path):
                return path
        return model_name

    def image_callback(self, msg):
        self.latest_image_msg = msg
        self.frame_ok += 1

    def process_latest_frame(self):
        msg = self.latest_image_msg
        if msg is None:
            return
        self.latest_image_msg = None

        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        elapsed = time.time() - self.start_time
        results = self.model.track(
            frame,
            persist=True,
            classes=[0],
            conf=0.5,
            imgsz=self.yolo_imgsz,
            tracker='botsort.yaml',
            verbose=False,
        )
        self.infer_count += 1

        detection_count = sum(len(result.boxes) for result in results)
        if detection_count == 0:
            self.log_status('No person detected.')
            self.show_debug_frame(frame, detection_count)
            return

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                person_img = frame[y1:y2, x1:x2]
                if person_img.size == 0:
                    continue

                h, w = person_img.shape[:2]
                roi_img = person_img[
                    int(h * 0.2):int(h * 0.8),
                    int(w * 0.25):int(w * 0.75),
                ]
                if roi_img.size == 0:
                    continue

                hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
                hist = cv2.calcHist(
                    [hsv],
                    [0, 1],
                    None,
                    [180, 256],
                    [0, 180, 0, 256],
                )
                cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

                if self.is_learning and elapsed < self.learning_duration:
                    self.master_db.append(hist)
                    self.log_status(
                        'Learning target: '
                        f'{len(self.master_db)} samples, '
                        f'{int(self.learning_duration - elapsed)}s remaining.'
                    )
                    cv2.putText(
                        frame,
                        f'Learning... {int(self.learning_duration - elapsed)}s',
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                    )
                    continue

                if self.is_learning:
                    self.is_learning = False
                    self.get_logger().info(
                        'Learning complete. '
                        f'Starting target tracking with {len(self.master_db)} samples.'
                    )

                max_sim = 0.0
                for db_hist in self.master_db:
                    sim = cv2.compareHist(db_hist, hist, cv2.HISTCMP_CORREL)
                    if sim > max_sim:
                        max_sim = sim

                if max_sim > 0.85:
                    label = 'Master'
                    color = (0, 255, 0)
                    self.update_servo(frame, x1, y1, x2, y2)
                    yolo_confidence = float(box.conf[0]) if box.conf is not None else max_sim
                    self.publish_person_detection(frame, x1, x2, yolo_confidence)
                else:
                    label = 'Unknown'
                    color = (0, 0, 255)
                    self.log_status(
                        f'Person detected, but not target. similarity={max_sim:.2f}'
                    )

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    frame,
                    f'{label} {max_sim:.2f}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

        self.show_debug_frame(frame, detection_count)

    def show_debug_frame(self, frame, detection_count):
        mode = 'learning' if self.is_learning else 'tracking'
        status_text = (
            f'recv:{self.frame_ok} infer:{self.infer_count} '
            f'person:{detection_count} mode:{mode}'
        )
        cv2.putText(
            frame,
            status_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow('Pan-Tilt YOLO Person Debug', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            rclpy.shutdown()

    def update_servo(self, frame, x1, y1, x2, y2):
        # --- 1. 좌우(Pan) 오차 계산 ---
        center_x = (x1 + x2) / 2
        error_x = center_x - (frame.shape[1] / 2)

        # 좌우(Pan) 업데이트 (오차가 50 픽셀 이상일 때만)
        if abs(error_x) > 50:
            self.target_pan_angle -= error_x * 0.02
            self.target_pan_angle = max(0, min(180, self.target_pan_angle))

        # 스무딩 처리
        smoothing_factor = 0.1
        self.current_pan_angle += (self.target_pan_angle - self.current_pan_angle) * smoothing_factor

        # --- 2. 퍼블리시 (마이크로초 변환) ---
        pan_us = int(np.interp(self.current_pan_angle, [0, 180], [500, 2500]))

        self.servo_pan_pub.publish(Int32(data=pan_us))
        self.publish_pan_angle()

        self.log_status(
            f'Target tracked. err_X={error_x:.1f} | Pan={pan_us}us'
        )

    def log_status(self, message):
        now = time.time()
        if now - self.last_status_log < 1.0:
            return
        self.last_status_log = now
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = SmartCartTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
