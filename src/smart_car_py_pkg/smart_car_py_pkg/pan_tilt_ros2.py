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
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from ultralytics import YOLO


COCO_SKELETON = [
    (5, 6),
    (5, 7),
    (7, 9),
    (6, 8),
    (8, 10),
    (5, 11),
    (6, 12),
    (11, 12),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
]


class SmartCartTracker(Node):
    def __init__(self):
        super().__init__('smart_cart_tracker_pc')
        self.bridge = CvBridge()
        self.declare_parameter('process_period_sec', 0.08)
        self.declare_parameter('yolo_imgsz', 320)
        self.declare_parameter('lowered_tilt_threshold_us', 1500)
        self.declare_parameter('ankle_conf_threshold', 0.35)
        self.declare_parameter('pose_stationary_conf_threshold', 0.35)
        self.declare_parameter('pose_stationary_motion_threshold', 0.03)
        self.declare_parameter('pose_stationary_min_keypoints', 6)
        self.declare_parameter('pose_resume_conf_threshold', 0.35)
        self.declare_parameter('pose_resume_knee_hip_tolerance', 0.08)
        self.declare_parameter('pose_period_sec', 0.3)
        self.declare_parameter('learning_complete_sound_topic', '/learning_complete_sound_cmd')
        self.declare_parameter('learning_complete_sound_value', 1)
        self.declare_parameter('learning_complete_sound_duration_ms', 1000)
        self.declare_parameter('start_enabled', False)

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
        self.servo_tilt_pub = self.create_publisher(Int32, '/servo_tilt_cmd', 10)
        self.pan_angle_pub = self.create_publisher(
            Float64, '/pan_tilt/pan_angle', 10)
        self.learning_complete_sound_pub = self.create_publisher(
            Int32, self.get_parameter('learning_complete_sound_topic').value, 10)
        self.detection_pub = self.create_publisher(
            Float32MultiArray, '/person_detection', 10)
        self.debug_image_pub = self.create_publisher(
            CompressedImage, '/pan_tilt/debug_image/compressed', 10)
        self.ankle_detected_pub = self.create_publisher(
            Bool, '/ankle_detected', 10)
        self.ankle_debug_pub = self.create_publisher(
            Float32MultiArray, '/ankle_keypoints_debug', 10)
        self.pose_stationary_pub = self.create_publisher(
            Bool, '/pose_stationary_detected', 10)
        self.pose_stationary_debug_pub = self.create_publisher(
            Float32MultiArray, '/pose_stationary_debug', 10)
        self.pose_resume_pub = self.create_publisher(
            Bool, '/pose_resume_detected', 10)
        self.pose_resume_debug_pub = self.create_publisher(
            Float32MultiArray, '/pose_resume_debug', 10)
        self.tilt_sub = self.create_subscription(
            Int32, '/servo_tilt_cmd', self.tilt_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/gui_command', self.command_callback, 10)

        self.model = None
        self.model_path = self._resolve_model_path('yolov8s.pt')
        self.pose_model = None
        self.pose_model_path = self._resolve_model_path('yolov8n-pose.pt')
        self.master_db = []
        self.master_track_id = None
        self.track_id_counts = {}
        self.person_following_enabled = bool(
            self.get_parameter('start_enabled').value)
        self.is_learning = True
        self.start_time = time.time()
        self.learning_duration = 50
        self.initial_pan_us = 1500
        self.initial_tilt_us = 1650
        self.current_pan_angle = self._microseconds_to_angle(self.initial_pan_us)
        self.target_pan_angle = self.current_pan_angle

        self.last_status_log = 0.0
        self.frame_ok = 0
        self.infer_count = 0
        self.latest_image_msg = None
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.lowered_tilt_threshold_us = int(
            self.get_parameter('lowered_tilt_threshold_us').value)
        self.ankle_conf_threshold = float(
            self.get_parameter('ankle_conf_threshold').value)
        self.pose_stationary_conf_threshold = float(
            self.get_parameter('pose_stationary_conf_threshold').value)
        self.pose_stationary_motion_threshold = float(
            self.get_parameter('pose_stationary_motion_threshold').value)
        self.pose_stationary_min_keypoints = int(
            self.get_parameter('pose_stationary_min_keypoints').value)
        self.pose_resume_conf_threshold = float(
            self.get_parameter('pose_resume_conf_threshold').value)
        self.pose_resume_knee_hip_tolerance = float(
            self.get_parameter('pose_resume_knee_hip_tolerance').value)
        self.pose_period_sec = float(self.get_parameter('pose_period_sec').value)
        self.learning_complete_sound_value = int(
            self.get_parameter('learning_complete_sound_value').value)
        self.learning_complete_sound_duration_ms = int(
            self.get_parameter('learning_complete_sound_duration_ms').value)
        self.last_pose_time = 0.0
        self.previous_pose_points = None
        self.previous_pose_visible = None
        self.latest_pose_points = None
        self.latest_pose_conf = None
        self.latest_pose_stationary = False
        self.latest_pose_motion = float('inf')
        self.latest_pose_keypoint_count = 0
        self.latest_pose_resume = False
        self.latest_pose_left_knee_hip_delta = float('inf')
        self.latest_pose_right_knee_hip_delta = float('inf')
        self.current_tilt_us = self.initial_tilt_us
        self.initial_pose_start_time = time.time()
        self.initial_pose_duration_sec = 5.0
        self.initial_pose_timer = self.create_timer(
            0.1, self.publish_initial_pose)
        self.process_timer = self.create_timer(
            float(self.get_parameter('process_period_sec').value),
            self.process_latest_frame)

        if self.person_following_enabled:
            self.get_logger().info('PC tracker node started in person following mode.')
        else:
            self.get_logger().info(
                'PC tracker node started in navigation mode. '
                'Waiting for CMD_SET_MODE_PERSON_FOLLOWING to start learning/tracking.'
            )

    def command_callback(self, msg):
        if msg.data == 'CMD_SET_MODE_PERSON_FOLLOWING':
            self.person_following_enabled = True
            self.reset_learning()
            self.get_logger().info('Person following mode enabled. Learning restarted.')
        elif msg.data == 'CMD_SET_MODE_NAVIGATION':
            self.person_following_enabled = False
            self.latest_image_msg = None
            self.get_logger().info('Navigation mode enabled. YOLO tracking paused.')

    def reset_learning(self):
        self.master_db = []
        self.master_track_id = None
        self.track_id_counts = {}
        self.is_learning = True
        self.start_time = time.time()
        self.last_pose_time = 0.0
        self.previous_pose_points = None
        self.previous_pose_visible = None
        self.latest_pose_points = None
        self.latest_pose_conf = None
        self.latest_pose_stationary = False
        self.latest_pose_motion = float('inf')
        self.latest_pose_keypoint_count = 0
        self.latest_pose_resume = False
        self.latest_pose_left_knee_hip_delta = float('inf')
        self.latest_pose_right_knee_hip_delta = float('inf')

    def publish_learning_complete_sound(self):
        self.learning_complete_sound_pub.publish(
            Int32(data=max(1, self.learning_complete_sound_duration_ms)))

    def _microseconds_to_angle(self, microseconds):
        return float(np.interp(microseconds, [500, 2500], [0, 180]))

    def _servo_angle_to_centered_rad(self, angle):
        return float(np.deg2rad(angle - 90.0))

    def publish_initial_pose(self):
        self.servo_pan_pub.publish(Int32(data=self.initial_pan_us))
        self.servo_tilt_pub.publish(Int32(data=self.initial_tilt_us))
        self.publish_pan_angle()
        if time.time() - self.initial_pose_start_time >= self.initial_pose_duration_sec:
            self.initial_pose_timer.cancel()
            self.get_logger().info(
                'Initial pan-tilt pose published for 5 seconds: '
                f'pan={self.initial_pan_us}us, tilt={self.initial_tilt_us}us')

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

    def tilt_callback(self, msg):
        self.current_tilt_us = int(msg.data)

    def is_camera_lowered(self):
        return self.current_tilt_us <= self.lowered_tilt_threshold_us

    def get_model(self):
        if self.model is None:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'YOLO model loaded: {self.model_path}')
        return self.model

    def publish_ankle_detected(self, detected):
        self.ankle_detected_pub.publish(Bool(data=bool(detected)))

    def publish_ankle_debug(self, detected, left_conf, right_conf):
        msg = Float32MultiArray()
        msg.data = [
            1.0 if detected else 0.0,
            float(left_conf),
            float(right_conf),
            float(self.ankle_conf_threshold),
            1.0 if self.is_camera_lowered() else 0.0,
        ]
        self.ankle_debug_pub.publish(msg)

    def publish_pose_stationary_detected(self, detected):
        self.pose_stationary_pub.publish(Bool(data=bool(detected)))

    def publish_pose_stationary_debug(self, detected, motion, keypoint_count):
        msg = Float32MultiArray()
        msg.data = [
            1.0 if detected else 0.0,
            float(motion),
            float(keypoint_count),
            float(self.pose_stationary_motion_threshold),
            float(self.pose_stationary_min_keypoints),
        ]
        self.pose_stationary_debug_pub.publish(msg)

    def publish_pose_resume_detected(self, detected):
        self.pose_resume_pub.publish(Bool(data=bool(detected)))

    def publish_pose_resume_debug(self, detected, left_delta, right_delta):
        msg = Float32MultiArray()
        msg.data = [
            1.0 if detected else 0.0,
            float(left_delta),
            float(right_delta),
            float(self.pose_resume_knee_hip_tolerance),
        ]
        self.pose_resume_debug_pub.publish(msg)

    def get_pose_model(self):
        if self.pose_model is None:
            self.pose_model = YOLO(self.pose_model_path)
            self.get_logger().info(f'Pose model loaded: {self.pose_model_path}')
        return self.pose_model

    def _normalize_pose_points(self, keypoints_xy, keypoints_conf):
        visible = keypoints_conf >= self.pose_stationary_conf_threshold
        if np.count_nonzero(visible) < self.pose_stationary_min_keypoints:
            return None, visible

        visible_points = keypoints_xy[visible]
        min_xy = np.min(visible_points, axis=0)
        max_xy = np.max(visible_points, axis=0)
        scale = float(np.linalg.norm(max_xy - min_xy))
        if scale <= 1e-6:
            return None, visible

        return (keypoints_xy - min_xy) / scale, visible

    def _evaluate_pose_stationary(self, keypoints_xy, keypoints_conf):
        normalized_points, visible = self._normalize_pose_points(
            keypoints_xy, keypoints_conf)
        if normalized_points is None:
            self.previous_pose_points = None
            self.previous_pose_visible = None
            return False, float('inf'), 0

        if self.previous_pose_points is None or self.previous_pose_visible is None:
            self.previous_pose_points = normalized_points
            self.previous_pose_visible = visible
            return False, float('inf'), int(np.count_nonzero(visible))

        common_visible = np.logical_and(visible, self.previous_pose_visible)
        common_count = int(np.count_nonzero(common_visible))
        if common_count < self.pose_stationary_min_keypoints:
            self.previous_pose_points = normalized_points
            self.previous_pose_visible = visible
            return False, float('inf'), common_count

        deltas = normalized_points[common_visible] - self.previous_pose_points[common_visible]
        motion = float(np.mean(np.linalg.norm(deltas, axis=1)))
        stationary = motion <= self.pose_stationary_motion_threshold

        self.previous_pose_points = normalized_points
        self.previous_pose_visible = visible
        return stationary, motion, common_count

    def _pose_scale(self, keypoints_xy, keypoints_conf, threshold):
        visible = keypoints_conf >= threshold
        if np.count_nonzero(visible) < 2:
            return 0.0

        visible_points = keypoints_xy[visible]
        min_xy = np.min(visible_points, axis=0)
        max_xy = np.max(visible_points, axis=0)
        return float(np.linalg.norm(max_xy - min_xy))

    def _evaluate_pose_resume(self, keypoints_xy, keypoints_conf):
        scale = self._pose_scale(
            keypoints_xy,
            keypoints_conf,
            self.pose_resume_conf_threshold,
        )
        if scale <= 1e-6:
            return False, float('inf'), float('inf')

        # COCO indices: left hip/knee = 11/13, right hip/knee = 12/14.
        sides = [(11, 13), (12, 14)]
        deltas = []
        for hip_idx, knee_idx in sides:
            if (
                keypoints_conf[hip_idx] < self.pose_resume_conf_threshold or
                keypoints_conf[knee_idx] < self.pose_resume_conf_threshold
            ):
                deltas.append(float('inf'))
                continue

            hip_y = keypoints_xy[hip_idx][1]
            knee_y = keypoints_xy[knee_idx][1]
            deltas.append(float((knee_y - hip_y) / scale))

        left_delta, right_delta = deltas
        detected = (
            left_delta <= self.pose_resume_knee_hip_tolerance or
            right_delta <= self.pose_resume_knee_hip_tolerance
        )
        return detected, left_delta, right_delta

    def detect_pose_keypoints(self, frame):
        results = self.get_pose_model()(
            frame,
            conf=0.5,
            imgsz=self.yolo_imgsz,
            device='cuda:0',
            verbose=False,
        )
        best_left_conf = 0.0
        best_right_conf = 0.0
        best_person_xy = None
        best_person_conf = None
        best_visible_count = 0
        for result in results:
            if (
                result.keypoints is None or
                result.keypoints.conf is None or
                result.keypoints.xy is None
            ):
                continue
            confs = result.keypoints.conf.cpu().numpy()
            points = result.keypoints.xy.cpu().numpy()
            for person_xy, person_conf in zip(points, confs):
                if len(person_conf) <= 16:
                    continue
                best_left_conf = max(best_left_conf, float(person_conf[15]))
                best_right_conf = max(best_right_conf, float(person_conf[16]))
                visible_count = int(np.count_nonzero(
                    person_conf >= self.pose_stationary_conf_threshold))
                if visible_count > best_visible_count:
                    best_visible_count = visible_count
                    best_person_xy = person_xy
                    best_person_conf = person_conf

        if best_person_xy is None or best_person_conf is None:
            self.previous_pose_points = None
            self.previous_pose_visible = None
            self.latest_pose_points = None
            self.latest_pose_conf = None
            stationary_detected = False
            pose_motion = float('inf')
            pose_keypoint_count = 0
        else:
            self.latest_pose_points = best_person_xy
            self.latest_pose_conf = best_person_conf
            stationary_detected, pose_motion, pose_keypoint_count = (
                self._evaluate_pose_stationary(best_person_xy, best_person_conf)
            )
            pose_resume_detected, left_knee_hip_delta, right_knee_hip_delta = (
                self._evaluate_pose_resume(best_person_xy, best_person_conf)
            )
        if best_person_xy is None or best_person_conf is None:
            pose_resume_detected = False
            left_knee_hip_delta = float('inf')
            right_knee_hip_delta = float('inf')

        self.latest_pose_stationary = stationary_detected
        self.latest_pose_motion = pose_motion
        self.latest_pose_keypoint_count = pose_keypoint_count
        self.latest_pose_resume = pose_resume_detected
        self.latest_pose_left_knee_hip_delta = left_knee_hip_delta
        self.latest_pose_right_knee_hip_delta = right_knee_hip_delta

        ankle_detected = (
            best_left_conf >= self.ankle_conf_threshold or
            best_right_conf >= self.ankle_conf_threshold
        )
        return (
            ankle_detected,
            best_left_conf,
            best_right_conf,
            stationary_detected,
            pose_motion,
            pose_keypoint_count,
            pose_resume_detected,
            left_knee_hip_delta,
            right_knee_hip_delta,
        )

    def _get_track_id(self, box):
        if box.id is None:
            return None
        try:
            return int(box.id[0])
        except (TypeError, ValueError, IndexError):
            return None

    def _select_master_track_id(self):
        if not self.track_id_counts:
            return None
        return max(self.track_id_counts, key=self.track_id_counts.get)

    def _resolve_model_path(self, model_name):
        candidates = []
        try:
            share_dir = get_package_share_directory('smart_car_py_pkg')
            candidates.append(os.path.join(share_dir, 'models', model_name))
        except Exception:
            pass

        candidates.extend([
            os.path.join(os.path.dirname(__file__), model_name),
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

        if not self.person_following_enabled:
            cv2.putText(
                frame,
                'NAVIGATION MODE',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self.publish_debug_image(frame)
            return

        elapsed = time.time() - self.start_time
        now = time.time()
        if now - self.last_pose_time >= self.pose_period_sec:
            self.last_pose_time = now
            (
                ankle_detected,
                left_conf,
                right_conf,
                pose_stationary_detected,
                pose_motion,
                pose_keypoint_count,
                pose_resume_detected,
                left_knee_hip_delta,
                right_knee_hip_delta,
            ) = self.detect_pose_keypoints(frame)
            self.publish_ankle_detected(ankle_detected)
            self.publish_ankle_debug(ankle_detected, left_conf, right_conf)
            self.publish_pose_stationary_detected(pose_stationary_detected)
            self.publish_pose_stationary_debug(
                pose_stationary_detected,
                pose_motion,
                pose_keypoint_count,
            )
            self.publish_pose_resume_detected(pose_resume_detected)
            self.publish_pose_resume_debug(
                pose_resume_detected,
                left_knee_hip_delta,
                right_knee_hip_delta,
            )

        results = self.get_model().track(
            frame,
            persist=True,
            classes=[0],
            conf=0.5,
            imgsz=self.yolo_imgsz,
            tracker='botsort.yaml',
            device='cuda:0',
            verbose=False,
        )
        self.infer_count += 1

        detection_count = sum(len(result.boxes) for result in results)
        if detection_count == 0:
            self.log_status('No person detected.')
            self.show_debug_frame(frame, detection_count)
            return

        master_visible = (
            self.master_track_id is not None and
            any(
                self._get_track_id(box) == self.master_track_id
                for result in results
                for box in result.boxes
            )
        )

        for result in results:
            for box in result.boxes:
                track_id = self._get_track_id(box)
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
                    if track_id is not None:
                        self.track_id_counts[track_id] = (
                            self.track_id_counts.get(track_id, 0) + 1
                        )
                    self.log_status(
                        'Learning target: '
                        f'{len(self.master_db)} samples, '
                        f'track_id={track_id}, '
                        f'{int(self.learning_duration - elapsed)}s remaining.'
                    )
                    cv2.putText(
                        frame,
                        f'{int(self.learning_duration - elapsed)}s',
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                    )
                    continue

                if self.is_learning:
                    self.is_learning = False
                    self.master_track_id = self._select_master_track_id()
                    self.get_logger().info(
                        'Learning complete. '
                        f'Starting target tracking with {len(self.master_db)} samples. '
                        f'master_track_id={self.master_track_id}'
                    )
                    self.publish_learning_complete_sound()

                max_sim = 0.0
                for db_hist in self.master_db:
                    sim = cv2.compareHist(db_hist, hist, cv2.HISTCMP_CORREL)
                    if sim > max_sim:
                        max_sim = sim

                is_master_track = (
                    self.master_track_id is not None and
                    track_id == self.master_track_id
                )
                is_reidentified_master = (
                    not master_visible and
                    self.master_track_id is not None and
                    track_id is not None and
                    max_sim > 0.8
                )
                if is_master_track or is_reidentified_master:
                    label = 'Master'
                    color = (0, 255, 0)
                    if is_reidentified_master:
                        self.master_track_id = track_id
                        self.get_logger().info(
                            f'Master track id reidentified: {self.master_track_id}')
                    self.update_servo(frame, x1, y1, x2, y2)
                    yolo_confidence = float(box.conf[0]) if box.conf is not None else max_sim
                    self.publish_person_detection(frame, x1, x2, yolo_confidence)
                else:
                    label = 'Unknown'
                    color = (0, 0, 255)
                    self.log_status(
                        'Person detected, but not target. '
                        f'track_id={track_id}, master_track_id={self.master_track_id}, '
                        f'similarity={max_sim:.2f}'
                    )

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    frame,
                    f'{label} id:{track_id} sim:{max_sim:.2f}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

        self.show_debug_frame(frame, detection_count)

    def get_learning_status_text(self, detection_count):
        if self.is_learning:
            remaining_s = max(0, int(self.learning_duration - (time.time() - self.start_time)))
            return f'person:{detection_count} learning:{remaining_s}s'
        return f'person:{detection_count} learning complete'

    def draw_pose_overlay(self, frame):
        if self.latest_pose_points is None or self.latest_pose_conf is None:
            cv2.putText(
                frame,
                'pose: none',
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 180, 255),
                2,
                cv2.LINE_AA,
            )
            return

        visible = self.latest_pose_conf >= self.pose_stationary_conf_threshold
        for start_idx, end_idx in COCO_SKELETON:
            if (
                start_idx >= len(self.latest_pose_points) or
                end_idx >= len(self.latest_pose_points) or
                not visible[start_idx] or
                not visible[end_idx]
            ):
                continue
            start = tuple(np.round(self.latest_pose_points[start_idx]).astype(int))
            end = tuple(np.round(self.latest_pose_points[end_idx]).astype(int))
            cv2.line(frame, start, end, (255, 180, 0), 2, cv2.LINE_AA)

        for index, point in enumerate(self.latest_pose_points):
            if index >= len(visible) or not visible[index]:
                continue
            center = tuple(np.round(point).astype(int))
            cv2.circle(frame, center, 4, (0, 255, 255), -1, cv2.LINE_AA)

        motion_text = (
            f"pose stationary:{int(self.latest_pose_stationary)} "
            f"motion:{self.latest_pose_motion:.3f} "
            f"kpt:{self.latest_pose_keypoint_count}"
        )
        resume_text = (
            f"resume:{int(self.latest_pose_resume)} "
            f"knee_hip L:{self.latest_pose_left_knee_hip_delta:.2f} "
            f"R:{self.latest_pose_right_knee_hip_delta:.2f}"
        )
        cv2.putText(
            frame,
            motion_text,
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            resume_text,
            (10, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def show_debug_frame(self, frame, detection_count):
        status_text = self.get_learning_status_text(detection_count)
        self.draw_pose_overlay(frame)
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
        self.publish_debug_image(frame)
        cv2.imshow('Pan-Tilt YOLO Person Debug', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            rclpy.shutdown()

    def publish_debug_image(self, frame):
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'pan_tilt_debug'
            self.debug_image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'Debug image publish failed: {exc}')

    def update_servo(self, frame, x1, y1, x2, y2):
        # --- 1. 좌우(Pan) 오차 계산 ---
        center_x = (x1 + x2) / 2
        error_x = center_x - (frame.shape[1] / 2)

        # 좌우(Pan) 업데이트 (오차가 50 픽셀 이상일 때만)
        if abs(error_x) > 50:
            self.target_pan_angle -= error_x * 0.02
            self.target_pan_angle = max(0, min(180, self.target_pan_angle))

        # 스무딩 처리
        smoothing_factor = 0.25
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
