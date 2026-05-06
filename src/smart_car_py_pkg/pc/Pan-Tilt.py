import cv2
import numpy as np
import os
import rclpy
import time
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from ultralytics import YOLO


class SmartCartTracker(Node):
    def __init__(self):
        super().__init__('smart_cart_tracker_pc')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/webcam2/image_raw/compressed',
            self.image_callback,
            10,
        )
        self.servo_pub = self.create_publisher(Int32, '/servo_cmd', 10)

        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.model = YOLO(os.path.join(base_dir, 'yolov8n.pt'))
        self.master_db = []
        self.is_learning = True
        self.start_time = time.time()
        self.learning_duration = 20
        self.current_servo_angle = 90.0
        self.target_servo_angle = 90.0
        self.last_status_log = 0.0
        self.frame_ok = 0
        self.infer_count = 0

        self.get_logger().info(
            'PC tracker node started. Waiting for Raspberry Pi camera images...'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        self.frame_ok += 1
        elapsed = time.time() - self.start_time
        results = self.model.track(
            frame,
            persist=True,
            classes=[0],
            conf=0.5,
            imgsz=640,
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

                hsv = cv2.cvtColor(person_img, cv2.COLOR_BGR2HSV)
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

                if max_sim > 0.7:
                    label = 'Master'
                    color = (0, 255, 0)
                    self.update_servo(frame, x1, x2)
                else:
                    label = 'Unknown'
                    color = (0, 0, 255)
                    self.log_status(f'Person detected, but not target. similarity={max_sim:.2f}')

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

    def update_servo(self, frame, x1, x2):
        center_x = (x1 + x2) / 2
        error = center_x - (frame.shape[1] / 2)

        if abs(error) <= 50:
            return

        self.target_servo_angle -= error * 0.02
        self.target_servo_angle = max(0, min(180, self.target_servo_angle))

        smoothing_factor = 0.1
        self.current_servo_angle += (
            self.target_servo_angle - self.current_servo_angle
        ) * smoothing_factor

        microsec = int(np.interp(self.current_servo_angle, [0, 180], [500, 2500]))
        self.servo_pub.publish(Int32(data=microsec))
        self.log_status(
            f'Target tracked. error={error:.1f}px, servo={microsec}us, '
            f'angle={self.current_servo_angle:.1f}deg.'
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


if __name__ == '__main__':
    main()
