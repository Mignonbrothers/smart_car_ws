# 🛒 Smart Cart — TurtleBot3 기반 자율주행 쇼핑 로봇

> **AI · 자율주행 · IoT 융합형 차세대 지능형 리테일 플랫폼**
> YOLOv8n + BoT-SORT 기반 사용자 추종, Nav2 자율주행, ROSbridge 연동 무인 결제 시스템.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blueviolet)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![C++](https://img.shields.io/badge/C++-17-00599C)
![YOLOv8](https://img.shields.io/badge/YOLOv8-n%2Fs%2Fpose-success)
![TurtleBot3](https://img.shields.io/badge/Robot-TurtleBot3%20Waffle%20Pi-red)
![License](https://img.shields.io/badge/License-Educational-lightgrey)

---

프로젝트 개요
Smart Cart는 기존 수동 쇼핑 카트의 한계(계산대 병목, 매장 탐색 피로)를 해결하기 위해 설계된 자율주행 쇼핑 로봇입니다. TurtleBot3 Waffle Pi 플랫폼 위에 팬틸트 웹캠, 2D LiDAR, ESP32 서보 제어기, Raspberry Pi 4를 통합하여 다음 3가지 핵심 시나리오를 수행합니다.

#	시나리오	핵심 기술
1	사람 추종 (Person Following)	YOLOv8n + BoT-SORT + HSV 색상 히스토그램 Re-ID + YOLOv8n-pose 제스처
2	목적지 자율주행 (Goal Navigation)	Nav2 (AMCL + DWB) + 사전 SLAM 맵 + Action 클라이언트
3	무인 물품 인식 / 결제	커스텀 학습 YOLO (best.pt) + Flask 웹 UI + QR 결제
설계 원칙 — 인식 계층(Python)과 제어 계층(C++)을 ROS 2 토픽으로 분리하고, 연산은 워크스테이션에서 수행합니다. Raspberry Pi 4는 센서 데이터를 발행하고, 로봇으로 돌아가는 제어 명령은 /cmd_vel 입니다.

핵심 기능
1. 사용자 추종 주행 (Person Following)
YOLOv8n + BoT-SORT : model.track(persist=True, tracker='botsort.yaml')로 사람 탐지 및 Track-ID 부여. BoT-SORT가 프레임 간 ID 연속성을 담당합니다.
50초 학습 모드 (learning_duration = 50) : 초기 사용자 의류 HSV(H-S 2D) 히스토그램을 마스터 DB(master_db)에 등록
Re-ID 매칭 : cv2.compareHist(..., HISTCMP_CORREL) 유사도 0.8 초과 시 마스터 트랙 재할당 → 가림(occlusion) 후 자동 재인식
YOLOv8n-Pose (17 keypoints) : 무릎(13,14)·골반(11,12) 좌표를 BBox 기준으로 정규화하여
정지 동작(pose_stationary_detected) → 3초 유지 시 자동 정지
재개 동작(무릎 들기 = pose_resume_detected) → 자동 재추종
거리 측정 : YOLO BBox 크기로 거리를 추정하지 않고, 팬 서보가 향한 방향(/pan_tilt/pan_angle)의 LiDAR 값을 calculateDistanceInDirection()으로 직접 읽습니다. 인식은 방향만, 거리는 센서가 담당하는 구조입니다.
2. Nav2 기반 목적지 안내
사전 정의된 5개 노드 좌표 (map 프레임 기준, go_to_pose.cpp의 kDestinations) :

노드	라벨	좌표 (x, y)
home	충전소	(-0.219, 0.052)
sunscreen	선크림	(2.418, 0.119)
wet_tissue	물티슈	(2.375, -2.688)
stationery	문구류	(-0.119, -0.874)
toilet	화장실	(1.598, -1.285)
GUI 버튼 → /gui_command (std_msgs/String, CMD_NAV_TO_<destination>) → go_to_pose_cpp 노드가 Nav2 navigate_to_pose Action 호출. 액션 서버 대기 타임아웃은 30초입니다.

3. 장애물 회피 (Reactive Avoidance)
LiDAR 정면(0° 중심) ±45° 섹터 최소 거리 40cm 이하 감지 → 선속도 0, 회전만 수행
좌(+90° 중심) / 우(-90° 중심) ±45° 섹터 평균 거리 비교 → 넓은 방향으로 회전
벽 추종 모드 (P-제어, target 0.65m, kp 0.8, 선속도 0.08m/s)로 간격 유지하며 통과
정면 1.30m 이상 클리어 + 0.30s 홀드 → 추종 모드 복귀
정면은 최소거리, 좌우는 평균거리를 사용합니다. 좌우까지 최소거리로 판정하면 벽 모서리 하나에 과민 반응합니다. 복귀 조건의 0.30초 홀드는 LiDAR 단일 프레임 튐으로 회피↔추종이 진동하는 것을 막습니다.

4. 무인 물품 인식 / 결제
상품 인식 카메라 → 커스텀 학습 모델(best.pt)이 4종 상품(선크림/테이프/가위/물티슈) 인식
인식 결과를 Flask REST API (POST /api/add_item)로 전송. ROS 토픽이 아닌 HTTP로 연결하여 Flask 측이 ROS에 의존하지 않도록 분리
모바일 웹 UI(QR 코드 접속)에서 수량 조절 / 결제. /process_payment가 영수증을 렌더하고 payment_completed 플래그를 세우며, 카트 초기화는 /reset_cart 호출로 수행
동일 상품 연속 인식으로 수량이 자동 증가하는 것을 막기 위해 인식 등록 후 cooldown(기본 3.0s)을 두고, 수량은 수동 버튼으로 변경
5. 안전 제어 (Safe Stop)
person_follower의 입력(/scan, /person_detection, /pan_tilt/pan_angle)은 마지막 수신 시각을 추적하며, lost_timeout_s(0.20s) 초과 시 publishStop()으로 속도를 0으로 만듭니다. Wi-Fi 지연으로 토픽이 밀릴 때 로봇이 마지막 명령 속도로 계속 주행하는 것을 차단합니다.

시스템 아키텍처
┌─────────────────────────────────────────────────────────────────────────┐
│                         Ubuntu 22.04 PC (Workstation)                   │
│  ┌──────────────────┐   ┌──────────────────┐   ┌────────────────────┐   │
│  │ pan_tilt_ros2.py │   │  robot_gui.py    │   │  cart_gui.py       │   │
│  │  (YOLOv8n/Pose,  │   │  (DearPyGUI,     │   │  (Flask + QR,      │   │
│  │   BoT-SORT, Re-ID│   │   Nav2 GUI)      │   │   장바구니 UI)     │   │
│  └────────┬─────────┘   └────────┬─────────┘   └─────────┬──────────┘   │
│           │                      │                       │ HTTP :5000   │
│  ┌────────┴────────────┐ ┌───────┴─────────┐  ┌──────────┴─────────┐    │
│  │  person_follower    │ │  go_to_pose     │  │  ros2_cart_bridge  │    │
│  │  (C++, 10Hz 루프)   │ │  (Nav2 Action)  │  │  (YOLO best.pt)    │    │
│  └────────┬────────────┘ └───────┬─────────┘  └────────────────────┘    │
└───────────┼──────────────────────┼──────────────────────────────────────┘
            │ /cmd_vel             │ navigate_to_pose
            ▼ DDS (WiFi)           ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 TurtleBot3 Waffle Pi (Raspberry Pi 4)                   │
│   ┌──────────────┐  ┌────────────┐  ┌─────────────────────────────┐     │
│   │ LDS-02 LiDAR │  │  OpenCR    │  │  usb_cam ×2 (MJPEG 320×240) │     │
│   │   /scan      │  │ (Dynamixel)│  │  /webcam, /webcam2          │     │
│   └──────────────┘  └────────────┘  └─────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────────┘
                                  ▲ UDP 8889 (pan, tilt μs)
                                  │
                       ┌──────────┴─────────┐
                       │  ESP32 + 2× Servo  │
                       │  (Pan / Tilt 헤드) │
                       └────────────────────┘
워크스페이스 구조
smart_car_ws/
├── src/
│   ├── smart_car_cpp_pkg/                  # 실시간 제어 노드 (C++, ament_cmake)
│   │   ├── include/smart_car_cpp_pkg/
│   │   │   ├── person_follower.hpp         # 추종 컨트롤러 헤더
│   │   │   ├── go_to_pose.hpp              # Nav2 Action 클라이언트
│   │   │   └── kalman_filter.hpp           # 4D 상태(x,y,vx,vy) KF (미적용)
│   │   ├── src/pi4/
│   │   │   ├── person_follower.cpp         # 647 LOC, 추종+회피+포즈정지
│   │   │   ├── go_to_pose.cpp              # 193 LOC, 목적지 좌표 매핑
│   │   │   └── kalman_filter.cpp           # 147 LOC (미적용, '알려진 문제' 참조)
│   │   ├── config/person_follower.yaml     # 런타임 파라미터
│   │   └── launch/person_follower.launch.py
│   │
│   └── smart_car_py_pkg/                   # AI / GUI / 브릿지 (Python, ament_python)
│       ├── smart_car_py_pkg/
│       │   ├── pan_tilt_ros2.py            # 824 LOC, YOLO+Pose+팬틸트 추적
│       │   ├── servo_udp_bridge.py         # 77 LOC, ROS2 → ESP32 UDP
│       │   ├── turtlebot_sound_bridge.py   # 68 LOC, OpenCR Sound 브릿지
│       │   ├── cart_gui.py                 # 350 LOC, Flask 모바일 장바구니 UI
│       │   ├── ros2_cart_bridge.py         # 139 LOC, YOLO best.pt → Flask API
│       │   └── gui/
│       │       ├── robot_gui.py            # 596 LOC, DearPyGUI 관제 노드
│       │       ├── map_gui.py              # 551 LOC, MapManager (렌더/좌표변환)
│       │       ├── button_manager.py       # 248 LOC, 목적지 버튼 콜백
│       │       ├── gui_node.py             # robot_gui + Flask 스레드 동시 실행
│       │       └── maps/                   # map.pgm, map.yaml
│       ├── pc/
│       │   └── yolo_detect_ros2.py         # 104 LOC, YoloCompressedViewer (디버그)
│       ├── config/
│       │   ├── nav2_params.yaml            # Nav2 전체 스택 파라미터
│       │   └── webcam_params.yaml          # usb_cam ×2 (MJPEG)
│       ├── launch/
│       │   ├── webcam_launch.py            # 카메라 ×2 + republish ×2 + 서보 + 사운드
│       │   ├── person_tracking.launch.py   # 추종 모듈 단독 실행
│       │   ├── nav2_gui_localization.launch.py  # 통합 런치 (Nav2 + GUI + 추종)
│       │   └── cart_system.launch.py       # 결제 UI 런치
│       ├── product_images/                 # 상품 썸네일 4종 (suncream/tape/scissor/tissue)
│       └── setup.py                        # entry_points
│
└── README.md
노드 구성
노드 클래스	실행 이름	언어	역할
SmartCartTracker	pan_tilt_ros2	Python	YOLO 추적·Re-ID·제스처·팬 서보 각도 산출
PersonFollower	person_follower	C++	추종 주행 + LiDAR 장애물 회피 (10Hz)
GoToPoseNode	go_to_pose	C++	Nav2 NavigateToPose 액션 클라이언트 (노드명 go_to_pose_cpp)
RobotGUI	robot_gui	Python	DearPyGUI 관제 화면, 명령 발행 (노드명 robot_node)
Esp32ServoBridge	servo_udp_bridge	Python	서보 명령 → ESP32 UDP
TurtleBotSoundBridge	turtlebot_sound_bridge	Python	알림음 → OpenCR /sound
Ros2CartBridge	ros2_cart_bridge	Python	상품 YOLO 인식 → Flask REST
SmartCartServer	cart_gui	Python	Flask 결제 서버 (:5000)
YoloCompressedViewer	(직접 실행)	Python	YOLO 단독 디버그 뷰어
map_gui.py와 button_manager.py는 ROS 노드가 아니라 robot_gui.py가 사용하는 DearPyGUI 렌더링/콜백 모듈입니다.

Launch 파일별 구성
Launch	기동 노드
webcam_launch.py	usb_cam_node_exe ×2 (webcam, webcam2) + image_transport republish ×2 (raw→compressed) + servo_udp_bridge + turtlebot_sound_bridge
person_tracking.launch.py	pan_tilt_ros2 (노드명 smart_cart_tracker_pc) + person_follower
nav2_gui_localization.launch.py	nav2_bringup/bringup_launch.py + robot_gui + go_to_pose + pan_tilt_ros2 + person_follower
cart_system.launch.py	cart_gui + ros2_cart_bridge
하드웨어 / 소프트웨어 환경
항목	사양
로봇 베이스	TurtleBot3 Waffle Pi
컨트롤러	OpenCR 1.0 (Dynamixel XM430-W210)
SBC	Raspberry Pi 4 (4GB, Ubuntu 22.04 Server)
LiDAR	Robotis LDS-02
카메라	USB Webcam ×2 (MJPEG 320×240, 15fps)
팬틸트	ESP32 + Servo ×2 (Pan / Tilt, μs 단위 제어)
워크스테이션	Ubuntu 22.04 + NVIDIA GPU (CUDA 추론)
ROS 2 배포판	Humble Hawksbill
AI 모델	YOLOv8n (추적) / YOLOv8n-pose (제스처) / best.pt (상품 4종)
빌드 & 실행
1. 환경 준비
# 시스템 의존성
sudo apt update && sudo apt install -y \
    ros-humble-nav2-bringup ros-humble-usb-cam \
    ros-humble-image-transport-plugins ros-humble-turtlebot3-msgs \
    ros-humble-cv-bridge python3-colcon-common-extensions

# Python 의존성 (workstation)
cd ~/smart_car_ws/src/smart_car_py_pkg
pip install -r requirements.txt
# ultralytics, opencv-python==4.10.0.84, flask, qrcode[pil], dearpygui, PyYAML
2. 워크스페이스 빌드
cd ~/smart_car_ws
colcon build --symlink-install
source install/setup.bash
--symlink-install 사용 시 Python 노드 코드 수정 후 재빌드 없이 즉시 반영됩니다.

3. YOLO 가중치 배치
가중치 파일은 .gitignore의 *.pt 규칙으로 저장소에 포함되어 있지 않습니다.

cp yolov8n.pt yolov8n-pose.pt best.pt src/smart_car_py_pkg/pc/
colcon build --symlink-install --packages-select smart_car_py_pkg
setup.py의 data_files가 pc/*.pt를 share/smart_car_py_pkg/models/로 설치하고, pan_tilt_ros2.py의 _resolve_model_path()와 ros2_cart_bridge.py의 resolve_model_path()가 설치 경로 → 소스 경로 순으로 탐색합니다.

4. 실행 시나리오
(A) 카메라 & 서보 브리지
ros2 launch smart_car_py_pkg webcam_launch.py \
    esp32_host:=192.168.0.42 esp32_port:=8889
usb_cam 2대 + image_transport republisher(raw → compressed) + servo_udp_bridge + turtlebot_sound_bridge를 함께 띄웁니다.

(B) 사람 추종 단독 실행
ros2 launch smart_car_py_pkg person_tracking.launch.py
pan_tilt_ros2 + person_follower만 기동합니다. Nav2와 GUI는 포함되지 않습니다.

(C) Nav2 + GUI 통합 실행 (메인 시나리오)
ros2 launch smart_car_py_pkg nav2_gui_localization.launch.py \
    map:=$(ros2 pkg prefix smart_car_py_pkg)/share/smart_car_py_pkg/gui/maps/map.yaml \
    use_sim_time:=false
nav2_bringup의 bringup_launch.py를 포함하고, robot_gui · go_to_pose · pan_tilt_ros2 · person_follower를 함께 기동합니다.

(D) 무인 결제 시스템
ros2 launch smart_car_py_pkg cart_system.launch.py \
    image_topic:=/webcam/image_raw/compressed \
    confidence:=0.6 cooldown:=3.0
# → 브라우저: http://<PC IP>:5000  (모바일 QR 접속)
ROS 2 토픽 / 액션 인터페이스
카메라 토픽 배치
토픽	구독 노드	비고
/webcam2/image_raw/compressed	pan_tilt_ros2 (사람 추종), robot_gui (라이브 뷰)	두 노드 모두 하드코딩 구독
/webcam/image_raw/compressed	ros2_cart_bridge (상품 인식)	image_topic 파라미터 기본값
webcam_params.yaml은 webcam(/dev/video1), webcam2(/dev/video3)를 정의하고, webcam_launch.py가 image_raw를 각각 /webcam/image_raw, /webcam2/image_raw로 remap한 뒤 image_transport republish로 compressed 토픽을 생성합니다.

핵심 토픽 맵
토픽	타입	방향	설명
/webcam2/image_raw/compressed	sensor_msgs/CompressedImage	카메라→PC	사용자 추종 카메라 (MJPEG)
/webcam/image_raw/compressed	sensor_msgs/CompressedImage	카메라→PC	상품 인식 카메라
/person_detection	std_msgs/Float32MultiArray	pan_tilt→follower	[center_x, frame_width, confidence]
/pan_tilt/pan_angle	std_msgs/Float64	pan_tilt→follower	deg2rad(angle - 90) — 중앙 90°가 0rad
/ankle_detected	std_msgs/Bool	pan_tilt→follower	발목 키포인트 감지
/pose_stationary_detected	std_msgs/Bool	pan_tilt→follower	사용자 정지 동작
/pose_resume_detected	std_msgs/Bool	pan_tilt→follower	사용자 재개 동작 (무릎 들기)
/scan	sensor_msgs/LaserScan	LiDAR→follower, robot_gui	LDS-02 스캔
/cmd_vel	geometry_msgs/Twist	follower→OpenCR	주행 명령
/servo_pan_cmd, /servo_tilt_cmd	std_msgs/Int32	pan_tilt→bridge	서보 μs (500–2500)
/learning_complete_sound_cmd	std_msgs/Int32	pan_tilt→sound_bridge	학습 완료 알림
/sound	turtlebot3_msgs/Sound	sound_bridge→OpenCR	OpenCR 부저
/gui_command	std_msgs/String	GUI→follower, go_to_pose, pan_tilt	모드/목적지 명령
/initialpose	geometry_msgs/PoseWithCov...	GUI→AMCL	수동 초기 위치 설정
/robot_status	std_msgs/String	→robot_gui	상태 오버레이
/battery_state	sensor_msgs/BatteryState	→robot_gui	배터리 표시
GUI 명령어 프로토콜 (/gui_command)
명령	동작
CMD_SET_MODE_PERSON_FOLLOWING	사람 추종 모드 활성화, Nav2 goal 취소(async_cancel_all_goals), YOLO 학습 50초 재시작
CMD_SET_MODE_NAVIGATION	추종 정지, 목적지 명령 수신 대기
CMD_NAV_TO_toilet / _sunscreen / _wet_tissue / _stationery / _home	사전 정의 좌표로 Nav2 Action 송신
person_follower와 go_to_pose가 모두 /cmd_vel 계통을 제어하므로 동시 활성화되면 안 됩니다. 두 노드가 /gui_command를 함께 구독하고 플래그(person_following_enabled_, navigation_enabled_)를 반대로 전환하여 배타 제어합니다. navigation_enabled_가 false인 동안 들어온 목적지 명령은 무시하고 로그만 남깁니다.

Nav2
액션	타입	클라이언트
/navigate_to_pose	nav2_msgs/action/NavigateToPose	go_to_pose_cpp
config/nav2_params.yaml은 Nav2 전체 스택을 구성합니다.

서버	주요 설정
amcl	nav2_amcl::DifferentialMotionModel
controller_server	FollowPath → dwb_core::DWBLocalPlanner, SimpleProgressChecker, SimpleGoalChecker
local_costmap	InflationLayer, VoxelLayer
global_costmap	StaticLayer, InflationLayer
그 외	bt_navigator, planner_server, smoother_server, behavior_server, map_server, map_saver, waypoint_follower, velocity_smoother
실시간 파라미터 튜닝
person_follower는 declare_parameter + 파라미터 콜백(onParameterUpdate)을 구현하여 재빌드 없이 실시간 조정이 가능합니다.

# 추종 거리 조정 (정지 / 추종 / 가속)
ros2 param set /person_follower stop_distance_m 1.20
ros2 param set /person_follower follow_distance_m 1.50
ros2 param set /person_follower normal_linear_velocity 0.12

# 회피 거리/속도
ros2 param set /person_follower obstacle_trigger_distance_m 0.35
ros2 param set /person_follower wall_follow_target_distance_m 0.60

# 또는 rqt_reconfigure GUI 사용
ros2 run rqt_reconfigure rqt_reconfigure
주요 파라미터 (config/person_follower.yaml)
파라미터	기본값	단위	설명
stop_distance_m	1.00	m	정지 거리
follow_distance_m	1.30	m	정상 추종 거리
far_distance_m	1.80	m	가속 추종 거리
normal_linear_velocity	0.10	m/s	일반 선속도
fast_linear_velocity	0.15	m/s	원거리 가속 선속도
max_angular_velocity	0.22	rad/s	최대 각속도
body_turn_kp	0.30	–	회전 P-게인
aligned_angle_threshold_rad	0.08	rad	정렬 완료 판정
realign_angle_threshold_rad	0.18	rad	재정렬 시작 판정
lost_timeout_s	0.20	s	검출 유실 판정 (Safe Stop)
min_detection_confidence	0.50	–	YOLO 추종 최소 신뢰도
pose_stationary_hold_s	3.00	s	정지 동작 유지 시간
pose_stationary_timeout_s	0.75	s	정지 신호 유효 시간
obstacle_trigger_distance_m	0.40	m	회피 트리거 거리
obstacle_front_half_width_rad	0.785398	rad	섹터 반각 (±45°, 3섹터 공용)
avoidance_linear_velocity	0.00	m/s	회피 회전 시 선속도 (제자리 회전)
avoidance_angular_velocity	0.20	rad/s	회피 회전 각속도
front_clear_distance_m	1.30	m	회피 해제 거리
front_clear_hold_s	0.30	s	회피 해제 판정 유지 시간
min_avoidance_active_s	0.80	s	회피 최소 유지 시간
wall_follow_linear_velocity	0.08	m/s	벽 추종 선속도
wall_follow_angular_velocity	0.15	rad/s	벽 추종 각속도
wall_follow_target_distance_m	0.65	m	벽 추종 목표 거리
wall_follow_kp	0.80	–	벽 추종 P-게인
wall_follow_front_stop_distance_m	0.60	m	벽 추종 중 전방 정지 거리
pan_tilt_ros2의 제스처 임계값은 노드 파라미터로 선언되어 있습니다: pose_stationary_motion_threshold(0.03), pose_resume_knee_hip_tolerance(0.08), pose_stationary_min_keypoints(6).

알고리즘 디테일
사용자 학습 (Re-ID Master Registration)
[t=0~50s] Learning Phase  (learning_duration = 50)
   ├── YOLOv8n.track(persist=True, tracker='botsort.yaml')
   ├── 각 사람 BBox 중앙 ROI  person_img[h*0.2:h*0.8, w*0.25:w*0.75]  → HSV 변환
   ├── cv2.calcHist([hsv], [0,1], None, [180,256], [0,180, 0,256])
   │     → cv2.normalize(hist, hist, 0, 1, NORM_MINMAX)
   ├── master_db.append(hist)  +  track_id_counts[id]++
   └── 50초 후 가장 빈번한 track_id 를 master_track_id 로 확정
[t>50s] Tracking Phase
   ├── 매 프레임: max_sim = max(compareHist(db_hist, hist, HISTCMP_CORREL) for db_hist in master_db)
   ├── master_track_id == 현재 track_id  → 'Master' (녹색)
   ├── 마스터 사라짐 + max_sim > 0.8     → Re-ID, master_track_id 갱신
   └── 그 외                              → 'Unknown' (적색, 무시)
BoT-SORT가 프레임 간 track_id를 유지하고, 가림으로 ID가 끊기면 색상 히스토그램이 복구하는 2단 구조입니다.

포즈 기반 정지/재개
정지(Pose Stationary) : 17개 키포인트를 BBox로 정규화 → 직전 프레임과 공통으로 가시인 키포인트가 pose_stationary_min_keypoints(6) 미만이면 판정 보류. 평균 변위가 pose_stationary_motion_threshold(0.03) 이하면 정지로 판정하고, 3초(pose_stationary_hold_s) 유지 시 추종 일시정지(pose_stop_latched_).
재개(Pose Resume) : COCO 인덱스 좌/우 무릎(13,14)과 골반(11,12)의 y차이가 pose_resume_knee_hip_tolerance(0.08) 이하면 "무릎 들기"로 판정 → 추종 재개.
팬틸트 서보 제어
BBox 중심 x → 화면 중심과 오차 (error_x = center_x - frame_width/2)
   ├── Dead Zone : |error_x| ≤ 50px 이면 서보 유지 (미세 흔들림 제거)
   ├── 초과 시    : target_pan_angle -= error_x * 0.02
   ├── 클램프     : max(0, min(180, target_pan_angle))
   ├── Smoothing : current_pan_angle += (target - current) * 0.25
   ├── PWM 변환   : np.interp(current_pan_angle, [0,180], [500,2500]) → μs
   ├── /servo_pan_cmd 발행    → servo_udp_bridge → ESP32 (UDP 8889)
   └── /pan_tilt/pan_angle 발행 → person_follower (주행 방향 기준)
Dead Zone과 Smoothing이 없으면 YOLO BBox의 프레임 단위 노이즈가 서보 떨림 → /pan_tilt/pan_angle 노이즈 → 주행 방향 노이즈로 그대로 전파됩니다. 서보 제어 토픽과 주행용 각도 토픽을 분리하여 주행 노드가 서보 구현을 알 필요가 없도록 했습니다.

servo_udp_bridge의 초기값은 pan 1500μs / tilt 1650μs이며, _microseconds_to_angle()은 np.interp(us, [500,2500], [0,180])로 역변환합니다.

제어 루프 우선순위 (person_follower.cpp::controlLoop)
100ms(10Hz) 타이머로 동작하며, 위에서 조건이 걸리면 즉시 return하여 하위 판단을 건너뜁니다.

1. person_following_enabled_ == false      → return (아무것도 발행 안 함)
2. /scan 미수신 또는 pan 각도 미수신        → publishStop()
3. 정면 섹터 최소거리 ≤ 0.40m               → makeAvoidanceCommand() → return
4. pose_stop_latched_                      → publishStop()
5. 검출 후 lost_timeout_s 경과              → publishStop()
6. LiDAR 거리가 유한하지 않음               → publishStop()
7. 정상                                     → linear = f(distance), angular = f(pan_angle)
기본 동작이 정지입니다. 입력 중 하나라도 신선하지 않으면 멈춥니다.

회피 동작 흐름 (person_follower.cpp::updateObstacleAvoidance)
   ┌──────────────┐  front ≤ 0.40m  ┌────────────────────┐
   │  FOLLOWING   │ ───────────────►│  AVOIDANCE_TURN    │
   │              │                 │  선속도 0, 제자리 회전 │
   └──────┬───────┘                 │  좌우 평균 비교 → 넓은 쪽│
          ▲                         └────────┬───────────┘
          │ front ≥ 1.30m                    │ pan 각도 정렬 완료
          │ 0.30s 홀드                        ▼
          │                         ┌────────────────────┐
          └─────────────────────────┤  WALL_FOLLOWING    │
                                    │  P(d=0.65m, kp=0.8)│
                                    │  선속도 0.08m/s     │
                                    └────────────────────┘
LiDAR 섹터 분할:

섹터	중심각	범위	연산	용도
Center	0.0	-45° ~ +45°	calculateMinDistanceInSector()	회피 트리거 / 해제
Left	+M_PI/2	+45° ~ +135°	calculateAverageDistanceInSector()	회피 방향 선택
Right	-M_PI/2	-45° ~ -135°	calculateAverageDistanceInSector()	회피 방향 선택
세 섹터 모두 반각(half-width)으로 obstacle_front_half_width_rad(0.785398rad = 45°)를 공유합니다. 이 파라미터 하나를 바꾸면 정면 트리거 폭과 좌우 탐색 폭이 함께 변합니다.

GUI / Web UI 흐름
DearPyGUI 관제 화면 (robot_gui.py)
robot_gui.py만 rclpy Node(노드명 robot_node)이고, map_gui.py·button_manager.py는 렌더링/콜백 모듈입니다.

라이브 카메라 (/webcam2/image_raw/compressed) 임베드
실시간 LiDAR Scan 폴라 플롯 (/scan)
AMCL /initialpose 드래그 입력 (맵 클릭 → 화살표)
TF 기반 로봇 트래킹 화살표 (0.1s 타이머). map→base_link → map→base_footprint → odom→base_link → odom→base_footprint 순으로 fallback 조회하여 AMCL 미기동 시에도 odom 기준으로 표시
배터리(/battery_state) / 상태(/robot_status) 오버레이
5개 목적지 버튼 + "사람추종 ↔ 목적지 이동" 토글
모듈	역할
robot_gui.py	ROS 노드. 구독(CompressedImage, LaserScan, BatteryState, String) + TF 리스너, 발행(/gui_command, /initialpose)
map_gui.py	MapManager — map.yaml 로드, 줌/휠 콜백, world↔pixel 변환, 로봇 pose 갱신, 목적지 마커, 경로 프리뷰
button_manager.py	목적지/명령 버튼 생성 및 콜백, CMD_NAV_TO_{target} 문자열 조립
map_gui.py의 경로 계획(_build_free_mask → _nearest_free_pixel → _plan_path_pixels → _simplify_path → _draw_dashed_line)은 화면 표시용 프리뷰입니다. 실제 주행 경로는 Nav2가 계산합니다.

gui_node.py는 SmartCartServer(Flask)를 데몬 스레드로 띄우고 robot_gui의 main()을 실행하여 관제 화면과 결제 서버를 한 프로세스에서 구동합니다.

모바일 결제 UI (cart_gui.py)
클래스	역할
CartManager	장바구니 상태, 수량 증감, 리셋, 변경 감지
SystemUtils	LAN IP 조회, QR 버퍼 생성
TemplateManager	Bootstrap 5.3.0 기반 HTML 템플릿
SmartCartServer	Flask 앱 + 라우트
엔드포인트	메서드	용도
/	GET	장바구니 UI
/qrcode	GET	접속용 QR 이미지
/api/status	GET	장바구니 상태 폴링
/api/add_item	POST	인식 결과 등록 (ros2_cart_bridge가 호출)
/api/update_qty	POST	수량 +/-
/process_payment	GET	영수증 렌더 + payment_completed 설정
/show_success, /thank_you	GET	결제 완료 화면
/reset_cart	GET	카트 초기화
인식 → 결제 파이프라인:

/webcam/image_raw/compressed
   → Ros2CartBridge: cv2.imdecode → YOLO(best.pt) 추론
   → confidence(0.6) / cooldown(3.0s) 필터 → normalize_class_name()
   → requests.post("http://127.0.0.1:5000/api/add_item")   # cart_api_url
   → CartManager.add_item()
   → 폰 브라우저 폴링(/api/status) → 장바구니 갱신
토픽 흐름 한눈에 보기
┌─[webcam2 (USB)]── /webcam2/image_raw/compressed ──► pan_tilt_ros2 ──┐
│                                                                     │
│  pan_tilt_ros2 ─► /person_detection ─────────────┐                  │
│  pan_tilt_ros2 ─► /pan_tilt/pan_angle ──────────┐│                  │
│  pan_tilt_ros2 ─► /ankle_detected ──────────────┤├──► person_follower
│  pan_tilt_ros2 ─► /pose_stationary_detected ────┤│                  │
│  pan_tilt_ros2 ─► /pose_resume_detected ────────┘│                  │
│                                                  │                  │
│  pan_tilt_ros2 ─► /servo_pan_cmd ──┐             │                  │
│  pan_tilt_ros2 ─► /servo_tilt_cmd ─┤             │                  │
│                                    ▼             ▼                  │
│                              servo_udp_bridge ───► ESP32 (UDP 8889) │
│                                                                     │
│  pan_tilt_ros2 ─► /learning_complete_sound_cmd ─► sound_bridge ─► /sound
│                                                                     │
│  [LDS-02] ── /scan ───────────────────────────► person_follower     │
│                                                       │             │
│                                                       ▼             │
│                                                   /cmd_vel ─► OpenCR
│                                                                     │
│  robot_gui ─► /gui_command ─┬─► go_to_pose_cpp ─► /navigate_to_pose │
│              /initialpose ──┼─► person_follower    (Nav2 Action)    │
│                    │        └─► pan_tilt_ros2                       │
│                    ▼                                                │
│                  AMCL                                               │
│                                                                     │
│  [webcam] ── /webcam/image_raw/compressed ──► ros2_cart_bridge      │
│                                                       │ HTTP POST   │
│                                                       ▼             │
│                                                   cart_gui (:5000)  │
└─────────────────────────────────────────────────────────────────────┘
트러블슈팅
증상	원인 / 해결
카메라 영상이 안 나옴	ls /dev/video* → webcam_params.yaml의 video_device 번호 조정. v4l2-ctl -d /dev/videoN --list-formats-ext로 MJPEG 지원 확인. 디바이스 노드 2개 중 보통 낮은 번호가 capture 디바이스.
YOLO 추론 속도 저하	Raspberry Pi에서 YOLO 실행 금지. 워크스테이션 GPU(device='cuda:0')에서 실행, 이미지는 CompressedImage만 전송. imgsz 축소 권장.
제스처를 취해도 추종 시작이 몇 초 늦음	추적 모델이 yolov8s이면 제스처 인식까지 4초 이상 지연된다. yolov8n 사용. 아래 "추적 모델 스케일과 제스처 반응 지연" 참조.
추종 시 좌우 헤맴	body_turn_kp 낮추기 (0.3 → 0.2). aligned_angle_threshold_rad(0.08) 키우기.
학습 후 'Master' 미인식	50초간 한 명만 카메라 정면 유지. min_detection_confidence 0.5 → 0.4 완화.
회피 후 추종 복귀 안 됨	front_clear_distance_m(1.30)과 front_clear_hold_s(0.30) 확인. /scan 수신 상태 점검.
회피/추종이 계속 진동	front_clear_hold_s 또는 min_avoidance_active_s(0.80)를 늘린다. LiDAR 단일 프레임 튐이 원인.
로봇이 아예 안 움직임	CMD_SET_MODE_PERSON_FOLLOWING 전송 여부 확인. person_following_enabled_가 false면 controlLoop이 즉시 return한다.
Nav2 goal 거부됨	GUI에서 CMD_SET_MODE_NAVIGATION 먼저 전송. navigation_enabled_가 true여야 함.
ESP32 서보 무응답	servo_udp_bridge 로그에서 UDP 송신 IP/포트 확인. ESP32 펌웨어가 동일 포트(8889)에서 수신 대기 중인지 확인.
서보가 미세하게 계속 떨림	Dead Zone 임계(50px)를 키운다. Smoothing 계수(0.25)를 낮춘다.
turtlebot3_msgs/Sound 에러	sudo apt install ros-humble-turtlebot3-msgs
상품이 인식되는데 장바구니에 안 담김	cart_api_url(기본 http://127.0.0.1:5000/api/add_item)과 cart_gui 기동 여부 확인.
추적 모델 스케일과 제스처 반응 지연
추적 정밀도를 높이려고 추적 모델을 yolov8n → yolov8s로 올린 적이 있습니다. 추종 자체는 동작했지만, 제스처를 취한 뒤 추종이 시작되기까지 4초 이상 걸렸습니다. yolov8n에서는 즉시 반응합니다.

관측된 조건은 다음과 같습니다.

추론은 워크스테이션 GPU에서 수행 (device='cuda:0', imgsz=320)
처리 주기 process_period_sec = 0.08s (12.5Hz)
이미지 구독 QoS는 BEST_EFFORT / KEEP_LAST / depth=1 — 구독 측에 프레임이 적체되지 않음
process_latest_frame() 한 번에 pose 모델과 추적 모델이 순차 추론
정상 상태 추론 시간만으로는 12.5Hz 예산(80ms)을 넘길 이유가 없어 4초가 설명되지 않습니다. 모델 최초 로드 시점(get_model()은 첫 프레임에서 lazy load)과 CUDA 워밍업, 카메라 전송 경로의 지연을 후보로 두었으나 계측까지 완료하지 못했고, 반응성이 확보되는 yolov8n을 유지했습니다.

재현·계측이 필요한 항목입니다. 추론 소요 시간과 프레임 도착 간격을 각각 로깅하면 지연이 추론·전송·최초 로드 중 어디에서 발생하는지 분리할 수 있습니다.

알려진 문제 / 개선 과제
kalman_filter.cpp는 빌드에 포함되지만 호출되지 않습니다. CMakeLists.txt가 person_follower 타깃에 링크하지만 person_follower.cpp·.hpp 어디에서도 참조하지 않습니다. 가림 구간 궤적 유지용 4D(x, y, vx, vy) 필터로 작성했으나, BoT-SORT가 내부적으로 칼만 필터를 사용하므로 이중 적용할 이유가 없어 연결하지 않았습니다.
webcam_params.yaml의 주석과 실제 코드의 카메라 배치가 어긋나 있습니다. 주석은 webcam=사람 추종 / webcam2=바구니로 설명하지만, pan_tilt_ros2.py와 robot_gui.py는 /webcam2를 구독하고 ros2_cart_bridge의 기본 image_topic은 /webcam입니다. 실제 배선 기준으로 주석을 정리해야 합니다.
목적지 좌표가 go_to_pose.cpp의 kDestinations에 하드코딩되어 있습니다. 맵이 바뀌면 재빌드가 필요합니다. YAML 외부화 대상입니다.
person_follower의 모드/상태는 열거형이 아닌 불리언 플래그(person_following_enabled_, pose_stop_latched_, obstacle_avoidance_active_)로 관리됩니다. /cmd_vel을 다투는 노드가 둘뿐이라 문제되지 않았으나, 상태가 늘어나면 명시적 FSM으로 정리해야 합니다.
벽 추종은 한쪽 면만 참조합니다(wall_follow_side_). 양측이 모두 좁은 통로는 검증하지 않았습니다.
다수 로봇이 동일 Wi-Fi를 공유하면 이미지 스트리밍부터 지연됩니다. Timeout Safe Stop으로 안전 정지는 되지만 근본 해결은 아닙니다. 대역 분리(개별 핫스팟) 또는 해상도 하향이 필요합니다.
추적 모델을 yolov8s로 올렸을 때의 4초 지연은 원인이 규명되지 않았습니다(위 트러블슈팅 참조). 추론 소요 시간, 프레임 도착 간격, 최초 모델 로드 시점을 각각 로깅하는 계측이 선행되어야 합니다.
파이프라인 구간별 지연 계측 수단이 없습니다. 카메라 발행 → 전송 → 추론 → /cmd_vel 발행까지의 end-to-end latency를 측정할 방법이 없어, 성능 문제 발생 시 원인 구간을 특정하기 어렵습니다.
모델이 lazy load됩니다. get_model()·get_pose_model()이 첫 프레임에서 로드하므로 최초 추론에 로드와 CUDA 워밍업 비용이 함께 실립니다. 노드 기동 시 더미 추론으로 워밍업하면 첫 검출 지연을 없앨 수 있습니다.
_resolve_model_path()는 파일을 찾지 못하면 모델 이름을 그대로 반환합니다. 이 경우 ultralytics가 가중치를 자동 다운로드하므로, 의도치 않게 네트워크에 의존하고 첫 실행이 느려질 수 있습니다. 탐색 실패 시 명시적으로 에러를 내는 편이 안전합니다.
requirements.txt의 mysql-connector-python은 현재 사용하지 않습니다.
pc/ 디렉터리에 Pan-Tilt.py, servo_serial_bridge.py 등 초기 개발 과정의 파일이 커밋 이력에 남아 있습니다.

## 👥 팀 구성

| 역할 | 담당자 | 주요 기여 |
|---|---|---|
| 객체 탐지 / Re-ID | 이주석 | YOLOv8n + BoT-SORT 추적, Arduino 팬틸트, Git 형상관리 |
| 추종 제어 / Nav2 | 한수창 | 객체 추종 주행 로직, 경로 계획, Git 형상관리 |
| 추종 제어 / 팬틸트 | 신종현 | 객체 추종 주행 로직, Arduino 팬틸트 |
| 하드웨어 / DL 학습 | 성대현 | 하드웨어 통합, 상품 인식 YOLO 학습, 웹페이지 연동 |
| Web / GUI | 송훈정 | 스마트폰 장바구니 웹, 관제 GUI, 라벨링 |

> 본 프로젝트는 **로보테크 AI 자율주행 로봇 개발자 과정 (2026.05)** 1조의 14일 종합 프로젝트 산출물입니다.

---

## 🗓️ 개발 히스토리 (14일)

| 단계 | 기간 | 내용 |
|---|---|---|
| 기획·설계 | Day 1–2 | 요구사항 분석, 시스템 아키텍처 설계, 기술 스택 선정 |
| 환경 구성 | Day 3–5 | TurtleBot3 ↔ Pi4 ROS2 연동, 웹캠/LiDAR 드라이버 설치 |
| AI 모듈 개발 | Day 6–8 | YOLOv8 + BoT-SORT 추종 모듈, Re-ID, 팬틸트 연동 |
| 자율주행 구현 | Day 9–11 | SLAM 맵 구축, Nav2 자율주행, 장애물 회피 검증 |
| UI 개발 / 통합 | Day 12–14 | 결제 UI, ROSbridge 통신, 시연·버그 픽스 |

---

## 📜 라이선스

본 저장소는 교육 목적으로 작성되었습니다. 외부 사용 시 팀에 사전 문의 부탁드립니다.

---

## 🔗 참고 문헌

- [ROBOTIS TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Nav2 Documentation (Humble)](https://docs.nav2.org/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [BoT-SORT: Robust Associations Multi-Pedestrian Tracking](https://arxiv.org/abs/2206.14651)
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/)

---

<p align="center">
  <i>🤖 Built with ROS 2 Humble · YOLOv8 · Nav2 · TurtleBot3 · Made by Team 1 @ Robotech AI Course</i>
</p>
