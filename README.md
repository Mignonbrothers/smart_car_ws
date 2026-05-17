# 🛒 Smart Cart — TurtleBot3 기반 자율주행 쇼핑 로봇

> **AI · 자율주행 · IoT 융합형 차세대 지능형 리테일 플랫폼**
> YOLOv8n + BoT-SORT 기반 사용자 추종, Nav2 자율주행, ROSbridge 연동 무인 결제 시스템

![ROS2](https://img.shields.io/badge/ROS2-Humble-blueviolet)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![C++](https://img.shields.io/badge/C++-17-00599C)
![YOLOv8](https://img.shields.io/badge/YOLOv8-n%2Fs%2Fpose-success)
![TurtleBot3](https://img.shields.io/badge/Robot-TurtleBot3%20Waffle%20Pi-red)
![License](https://img.shields.io/badge/License-Educational-lightgrey)

---

## 📌 프로젝트 개요

**Smart Cart**는 기존 수동 쇼핑 카트의 한계(계산대 병목, 매장 탐색 피로)를 해결하기 위해 설계된 **자율주행 쇼핑 로봇**입니다. TurtleBot3 Waffle Pi 플랫폼 위에 팬틸트 웹캠, 2D LiDAR, ESP32 서보 제어기, Raspberry Pi 4를 통합하여 다음 3가지 핵심 시나리오를 수행합니다.

| # | 시나리오 | 핵심 기술 |
|---|---|---|
| **1** | 🚶 **사람 추종 (Person Following)** | YOLOv8n + BoT-SORT + HSV 색상 히스토그램 Re-ID + Kalman Filter |
| **2** | 🗺️ **목적지 자율주행 (Goal Navigation)** | Nav2 (AMCL + DWB) + 사전 SLAM 맵 + Action 클라이언트 |
| **3** | 💳 **무인 물품 인식 / 결제** | 커스텀 학습 YOLO (`best.pt`) + Flask 웹 UI + QR 결제 |

> 💡 **기획 의도** : *"양손이 자유로운 Hands-Free 쇼핑 + 매장 동선 데이터 기반 마케팅 최적화"*

---

## 🎯 핵심 기능

### 1️⃣ 사용자 추종 주행 (Person Following)
- **YOLOv8n + BoT-SORT** : 사람 객체 탐지 및 Track-ID 부여
- **50초 학습 모드** : 초기 사용자 의류 HSV(H-S 2D) 히스토그램을 마스터 DB(`master_db`)에 등록
- **Re-ID 매칭** : `cv2.compareHist(HISTCMP_CORREL)` 유사도 0.8 이상 시 마스터 트랙 재할당 → 가림(occlusion) 후 자동 재인식
- **YOLOv8n-Pose (17 keypoints)** : 무릎(13,14)·골반(11,12) 좌표를 정규화하여
  - 정지 동작(`pose_stationary_detected`) → 3초 유지 시 자동 정지
  - 재개 동작(무릎 들기 = `pose_resume_detected`) → 자동 재추종
- **CMC + Kalman Filter** : 카메라 모션 보정 후 위치/속도 예측 → 0.03s 앞 예측으로 부드러운 추종

### 2️⃣ Nav2 기반 목적지 안내
사전 정의된 5개 노드 좌표 (map 프레임 기준) :

| 노드 | 라벨 | 좌표 (x, y) |
|---|---|---|
| `home` | 충전소 | (-0.219, 0.052) |
| `sunscreen` | 선크림 | (2.418, 0.119) |
| `wet_tissue` | 물티슈 | (2.375, -2.688) |
| `stationery` | 문구류 | (-0.119, -0.874) |
| `toilet` | 화장실 | (1.598, -1.285) |

GUI 버튼 → `/gui_command` (std_msgs/String, `CMD_NAV_TO_<destination>`) → `go_to_pose_cpp` 노드가 Nav2 `navigate_to_pose` Action 호출.

### 3️⃣ 장애물 회피 (Reactive Avoidance)
- LiDAR 정면 ±45° 섹터 최소 거리 **40 cm 이하** 감지 → 즉시 정지
- 좌/우 평균 거리 비교 → 넓은 방향으로 회전
- **벽 추종 모드** (P-제어, target=0.65m) 로 50–65cm 간격 유지하며 회피
- 정면 1.30 m 이상 클리어 + 0.30 s 홀드 → 추종 모드 복귀

### 4️⃣ 무인 결제 시스템 (Smart Cart UI)
- 바구니 상단 웹캠 (`/webcam/image_raw/compressed`) → 커스텀 학습 모델(`best.pt`)이 4종 상품(선크림/테이프/가위/물티슈) 인식
- 인식 결과를 Flask REST API (`POST /api/add_item`)로 전송
- 모바일 웹 UI(QR 코드 접속)에서 수량 조절 / 결제 → QR 결제 완료 시 카트 자동 리셋

---

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Ubuntu 22.04 PC (Workstation)                   │
│  ┌──────────────────┐   ┌──────────────────┐   ┌────────────────────┐   │
│  │ pan_tilt_ros2.py │   │  robot_gui.py    │   │  cart_gui.py       │   │
│  │  (YOLOv8s/Pose,  │   │  (DearPyGUI,     │   │  (Flask + QR,      │   │
│  │   BoT-SORT, Re-ID│   │   Nav2 GUI)      │   │   장바구니 UI)     │   │
│  └────────┬─────────┘   └────────┬─────────┘   └─────────┬──────────┘   │
│           │                      │                       │              │
│  ┌────────┴────────────┐ ┌───────┴─────────┐  ┌──────────┴─────────┐    │
│  │  person_follower    │ │  go_to_pose     │  │  ros2_cart_bridge  │    │
│  │  (C++ Control Loop) │ │  (Nav2 Action)  │  │  (YOLO best.pt)    │    │
│  └────────┬────────────┘ └───────┬─────────┘  └────────────────────┘    │
└───────────┼──────────────────────┼──────────────────────────────────────┘
            │ /cmd_vel             │ navigate_to_pose
            ▼ DDS (WiFi)           ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 TurtleBot3 Waffle Pi (Raspberry Pi 4)                   │
│   ┌──────────────┐  ┌────────────┐  ┌─────────────────────────────┐     │
│   │ LDS-02 LiDAR │  │  OpenCR    │  │  usb_cam (×2, MJPEG 320×240) │    │
│   │   /scan      │  │ (Dynamixel)│  │  /webcam/image_raw/comp.    │     │
│   └──────────────┘  └────────────┘  └─────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────────┘
                                  ▲ UDP 8889 (pan, tilt μs)
                                  │
                       ┌──────────┴─────────┐
                       │  ESP32 + 2× Servo  │
                       │  (Pan / Tilt 헤드) │
                       └────────────────────┘
```

---

## 📂 워크스페이스 구조

```
smart_car_ws-main/
├── src/
│   ├── smart_car_cpp_pkg/                  # 실시간 제어 노드 (C++)
│   │   ├── include/smart_car_cpp_pkg/
│   │   │   ├── person_follower.hpp         # 추종 컨트롤러 헤더
│   │   │   ├── go_to_pose.hpp              # Nav2 Action 클라이언트
│   │   │   └── kalman_filter.hpp           # 4D 상태(x,y,vx,vy) KF
│   │   ├── src/pi4/
│   │   │   ├── person_follower.cpp         # 647 LOC, 추종+회피+포즈정지
│   │   │   ├── go_to_pose.cpp              # 목적지 좌표 매핑
│   │   │   └── kalman_filter.cpp
│   │   ├── config/person_follower.yaml     # 런타임 파라미터
│   │   └── launch/person_follower.launch.py
│   │
│   └── smart_car_py_pkg/                   # AI / GUI / 브릿지 (Python)
│       ├── smart_car_py_pkg/
│       │   ├── pan_tilt_ros2.py            # 824 LOC, YOLO+Pose+팬틸트 추적
│       │   ├── servo_udp_bridge.py         # ROS2 ↔ ESP32 UDP
│       │   ├── turtlebot_sound_bridge.py   # OpenCR Sound 토픽 브릿지
│       │   ├── cart_gui.py                 # Flask 모바일 장바구니 UI
│       │   ├── ros2_cart_bridge.py         # YOLO best.pt → Flask API
│       │   └── gui/
│       │       ├── robot_gui.py            # DearPyGUI 관제 화면
│       │       ├── map_gui.py              # 맵 렌더 + AMCL 초기 포즈
│       │       └── button_manager.py       # 목적지 버튼 정의
│       ├── pc/
│       │   └── yolo_detect_ros2.py         # YOLO 단독 디버그 뷰어
│       ├── config/
│       │   ├── nav2_params.yaml            # AMCL + DWB + Costmap
│       │   └── webcam_params.yaml          # usb_cam ×2 (MJPEG)
│       └── launch/
│           ├── webcam_launch.py            # 카메라 ×2 + 서보 + 사운드
│           ├── person_tracking.launch.py   # 추종 모듈 단독 실행
│           ├── nav2_gui_localization.launch.py  # 통합 런치 (Nav2+GUI)
│           └── cart_system.launch.py       # 결제 UI 런치
│
└── README.md  ← 현재 문서
```

---

## 🔧 하드웨어 / 소프트웨어 환경

| 항목 | 사양 |
|---|---|
| **로봇 베이스** | TurtleBot3 Waffle Pi |
| **컨트롤러** | OpenCR 1.0 (Dynamixel XM430-W210) |
| **SBC** | Raspberry Pi 4 (4GB, Ubuntu 22.04 Server) |
| **LiDAR** | Robotis LDS-02 (360°, 12Hz) |
| **카메라** | USB Webcam ×2 (사용자 추종용 / 바구니 인식용) |
| **팬틸트** | ESP32 + Servo ×2 (Pan / Tilt, μs 단위 제어) |
| **워크스테이션** | Ubuntu 22.04 + NVIDIA GPU (CUDA 추론) |
| **ROS 2 배포판** | Humble Hawksbill |
| **AI 모델** | YOLOv8n (추적) / YOLOv8s (탐지) / YOLOv8n-pose / best.pt (상품 4종) |

---

## 🚀 빌드 & 실행

### 1. 환경 준비

```bash
# 시스템 의존성
sudo apt update && sudo apt install -y \
    ros-humble-nav2-bringup ros-humble-usb-cam \
    ros-humble-image-transport-plugins ros-humble-turtlebot3-msgs \
    ros-humble-cv-bridge python3-colcon-common-extensions

# Python 의존성 (workstation)
cd ~/smart_car_ws/src/smart_car_py_pkg
pip install -r requirements.txt
# ultralytics, opencv-python==4.10.0.84, flask, qrcode[pil], dearpygui, PyYAML
```

### 2. 워크스페이스 빌드

```bash
cd ~/smart_car_ws
colcon build --symlink-install
source install/setup.bash
```

> 💡 `--symlink-install` 사용 시 Python 노드 코드 수정 후 재빌드 없이 즉시 반영됩니다.

### 3. YOLO 가중치 배치

```bash
# 학습된 가중치를 패키지 내 pc/ 또는 share/.../models/ 경로에 배치
cp yolov8n.pt yolov8s.pt yolov8n-pose.pt best.pt \
   src/smart_car_py_pkg/pc/
colcon build --symlink-install --packages-select smart_car_py_pkg
```

`pan_tilt_ros2.py` → `ros2_cart_bridge.py` 가 자동으로 모델 경로를 탐색합니다.

### 4. 실행 시나리오

#### (A) 카메라 & 서보 브리지 (PC + 로봇 공용)
```bash
ros2 launch smart_car_py_pkg webcam_launch.py \
    esp32_host:=192.168.0.42 esp32_port:=8889
```

#### (B) 사람 추종 단독 실행
```bash
ros2 launch smart_car_py_pkg person_tracking.launch.py
```

#### (C) Nav2 + GUI 통합 실행 (메인 시나리오)
```bash
ros2 launch smart_car_py_pkg nav2_gui_localization.launch.py \
    map:=$(ros2 pkg prefix smart_car_py_pkg)/share/smart_car_py_pkg/gui/maps/map.yaml \
    use_sim_time:=false
```

#### (D) 무인 결제 시스템
```bash
ros2 launch smart_car_py_pkg cart_system.launch.py \
    image_topic:=/webcam/image_raw/compressed \
    confidence:=0.6 cooldown:=3.0
# → 브라우저: http://<로봇IP>:5000  (모바일 QR 접속)
```

---

## 🔗 ROS 2 토픽 / 액션 인터페이스

### 핵심 토픽 맵

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/webcam/image_raw/compressed` | sensor_msgs/CompressedImage | 카메라→PC | 사용자 추종 카메라 (MJPEG) |
| `/webcam2/image_raw/compressed` | sensor_msgs/CompressedImage | 카메라→PC | 바구니 상단 인식 카메라 |
| `/person_detection` | std_msgs/Float32MultiArray | pan_tilt→follower | `[center_x, frame_width, confidence]` |
| `/pan_tilt/pan_angle` | std_msgs/Float64 | pan_tilt→follower | 팬 각도 (rad, 중앙=0) |
| `/ankle_detected` | std_msgs/Bool | pan_tilt→follower | 발목 키포인트 감지 |
| `/pose_stationary_detected` | std_msgs/Bool | pan_tilt→follower | 사용자 정지 동작 |
| `/pose_resume_detected` | std_msgs/Bool | pan_tilt→follower | 사용자 재개 동작 (무릎 들기) |
| `/scan` | sensor_msgs/LaserScan | LiDAR→follower | LDS-02 360° 스캔 |
| `/cmd_vel` | geometry_msgs/Twist | follower→OpenCR | 주행 명령 |
| `/servo_pan_cmd`, `/servo_tilt_cmd` | std_msgs/Int32 | pan_tilt→bridge | 서보 μs (500–2500) |
| `/gui_command` | std_msgs/String | GUI→ALL | 모드/목적지 명령 |
| `/initialpose` | geometry_msgs/PoseWithCov... | GUI→AMCL | 수동 초기 위치 설정 |
| `/learning_complete_sound_cmd` | std_msgs/Int32 | pan_tilt→sound | OpenCR 부저 알림 |

### GUI 명령어 프로토콜 (`/gui_command`)
| 명령 | 동작 |
|---|---|
| `CMD_SET_MODE_PERSON_FOLLOWING` | 사람 추종 모드 활성화, Nav2 goal 취소, YOLO 학습 50초 재시작 |
| `CMD_SET_MODE_NAVIGATION` | 추종 정지, 목적지 명령 수신 대기 |
| `CMD_NAV_TO_toilet` / `_sunscreen` / `_wet_tissue` / `_stationery` / `_home` | 사전 정의 좌표로 Nav2 Action 송신 |

### Nav2 Action

| 액션 | 타입 | 클라이언트 |
|---|---|---|
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | `go_to_pose_cpp` |

---

## ⚙️ 실시간 파라미터 튜닝

모든 노드는 `declare_parameter` + 파라미터 콜백을 구현하여 **재빌드 없이 실시간 조정**이 가능합니다.

### person_follower 주요 파라미터

```bash
# 추종 거리 조정 (정지 / 추종 / 가속)
ros2 param set /person_follower stop_distance_m 1.20
ros2 param set /person_follower follow_distance_m 1.50
ros2 param set /person_follower normal_linear_velocity 0.12

# 회피 거리/속도
ros2 param set /person_follower obstacle_trigger_distance_m 0.35
ros2 param set /person_follower wall_follow_target_distance_m 0.60

# 또는 rqt_reconfigure GUI 사용
ros2 run rqt_reconfigure rqt_reconfigure
```

### 전체 파라미터 표 (`config/person_follower.yaml`)

| 파라미터 | 기본값 | 단위 | 설명 |
|---|---|---|---|
| `stop_distance_m` | 1.00 | m | 정지 거리 |
| `follow_distance_m` | 1.30 | m | 정상 추종 거리 |
| `far_distance_m` | 1.80 | m | 가속 추종 거리 |
| `normal_linear_velocity` | 0.10 | m/s | 일반 선속도 |
| `fast_linear_velocity` | 0.15 | m/s | 원거리 가속 선속도 |
| `max_angular_velocity` | 0.22 | rad/s | 최대 각속도 |
| `body_turn_kp` | 0.30 | – | 회전 P-게인 |
| `min_detection_confidence` | 0.50 | – | YOLO 추종 최소 신뢰도 |
| `pose_stationary_hold_s` | 3.00 | s | 정지 동작 유지 시간 |
| `obstacle_trigger_distance_m` | 0.40 | m | 회피 트리거 거리 |
| `obstacle_front_half_width_rad` | 0.785 | rad | 정면 섹터 반각 (±45°) |
| `wall_follow_target_distance_m` | 0.65 | m | 벽 추종 목표 거리 |
| `wall_follow_kp` | 0.80 | – | 벽 추종 P-게인 |

---

## 🧠 알고리즘 디테일

### 사용자 학습 (Re-ID Master Registration)

```
[t=0~50s] Learning Phase
   ├── YOLOv8n.track(persist=True, tracker='botsort.yaml')
   ├── 각 사람 BBox 중앙 ROI (h:20~80%, w:25~75%) → HSV 변환
   ├── 2D Histogram (180 bins H × 256 bins S) 계산 후 [0,1] 정규화
   ├── master_db.append(hist)  +  track_id_counts[id]++
   └── 50초 후 가장 빈번한 track_id 를 master_track_id 로 확정

[t>50s] Tracking Phase
   ├── 매 프레임: max_sim = max(compareHist(hist, db_hist, CORREL) for db_hist in master_db)
   ├── master_track_id == 현재 track_id  → 'Master' (녹색)
   ├── 마스터 사라짐 + max_sim > 0.8     → Re-ID, master_track_id 갱신
   └── 그 외                              → 'Unknown' (적색, 무시)
```

### 포즈 기반 정지/재개

- **정지(Pose Stationary)** : 17개 키포인트를 BBox로 정규화 → 직전 프레임 대비 평균 변위가 `pose_stationary_motion_threshold (0.03)` 이하 + 6개 이상 키포인트 가시 → 정지로 판정. 3초 유지 시 추종 일시정지(`pose_stop_latched_`).
- **재개(Pose Resume)** : COCO 인덱스 좌/우 무릎(13,14)과 골반(11,12)의 y차이가 `pose_resume_knee_hip_tolerance (0.08)` 이하면 "무릎 들기" 로 판정 → 추종 재개.

### 회피 상태머신 (`person_follower.cpp::updateObstacleAvoidance`)

```
   ┌──────────────┐  front<0.40m   ┌────────────────┐
   │ FOLLOWING    │ ───────────────►│ AVOIDANCE_TURN │
   └──────┬───────┘                 └────────┬───────┘
          │                                  │ pan 정렬 완료
          │ front>1.30 m  ≥ 0.30s             ▼
          │                          ┌────────────────┐
          └──────────────────────────┤ WALL_FOLLOWING │
                                     │ P(d=0.65m, kp=0.8)│
                                     └────────────────┘
```

### Kalman Filter (`kalman_filter.cpp`)
- 상태 벡터 : `[x, y, vx, vy]` (4D)
- 등속 모델, dt 가변
- 측정 잡음 r_, 프로세스 잡음 q_ 만 튜닝 → 가림(occlusion) 0.3s 이내 궤적 유지

---

## 🛰️ GUI / Web UI 흐름

### DearPyGUI 관제 화면 (`robot_gui.py`)
- 라이브 카메라 (`/webcam2/image_raw/compressed`) 임베드
- 실시간 LiDAR Scan 폴라 플롯
- AMCL `/initialpose` 드래그 입력 (맵 클릭 → 화살표)
- TF (`map → base_link`) 기반 로봇 트래킹 화살표
- 배터리 / WiFi RSSI / 모드 상태 오버레이
- 5개 목적지 버튼 + "사람추종 ↔ 목적지 이동" 토글

### 모바일 결제 UI (`cart_gui.py`)
- Flask + Bootstrap 5.3 (Amazon UI 컨셉)
- QR 코드 자동 생성 (`/qr` 엔드포인트) → 동일 LAN의 스마트폰으로 접속
- 자동 수량 증가 차단 (중복 인식 무시), 수동 +/- 버튼만 허용
- "결제하기" → QR 결제 완료 팝업

---

## 🩺 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| 카메라 영상이 안 나옴 | `ls /dev/video*` → `webcam_params.yaml` 의 `video_device` 번호 조정. `v4l2-ctl -d /dev/videoN --list-formats-ext` 로 MJPEG 지원 확인. |
| YOLO 추론 속도 < 5 FPS | Raspberry Pi에서 YOLO 실행 금지. 워크스테이션 GPU(`device='cuda:0'`)에서 실행, 이미지는 `CompressedImage`만 전송. `yolo_imgsz` 320 권장. |
| 추종 시 좌우 헤매 | `body_turn_kp` 낮추기 (0.3 → 0.2). `aligned_angle_threshold_rad` 키우기. |
| 학습 모드가 끝나도 'Master' 미인식 | 50초간 한 명만 카메라 정면 유지. `min_detection_confidence` 0.5 → 0.4 완화. |
| 회피 후 추종 복귀 안됨 | `front_clear_distance_m` (1.30 m) 와 `front_clear_hold_s` (0.30 s) 확인. LiDAR 데이터 `/scan` 수신 상태 점검. |
| Nav2 goal 거부됨 | GUI에서 `CMD_SET_MODE_NAVIGATION` 먼저 전송. `navigation_enabled_` 플래그가 `true` 여야 함. |
| ESP32 서보 무응답 | `servo_udp_bridge` 로그에서 UDP 송신 IP/포트 확인. ESP32 펌웨어가 동일 포트(8889)에서 수신 대기 중인지 확인. |
| `tilt` 명령이 누락 | OpenCR `/sound` 토픽처럼 메시지 타입(`turtlebot3_msgs/Sound`) 미설치 → `sudo apt install ros-humble-turtlebot3-msgs`. |

---

## 📊 토픽 흐름 한눈에 보기

```
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
│                              servo_udp_bridge ───► ESP32 (UDP)      │
│                                                                     │
│  [LDS-02] ── /scan ───────────────────────────► person_follower     │
│                                                       │             │
│                                                       ▼             │
│                                                   /cmd_vel ─► OpenCR
│                                                                     │
│  robot_gui ─► /gui_command ─► go_to_pose_cpp ─► /navigate_to_pose   │
│                                                  (Nav2 Action)      │
└─────────────────────────────────────────────────────────────────────┘
```

---

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
