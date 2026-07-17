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

## 개요

TurtleBot3 Waffle Pi에 팬틸트 웹캠, 2D LiDAR, ESP32 서보 제어기를 통합한 자율주행 쇼핑 카트입니다. 사용자를 따라다니다가, 터치 패널에서 목적지를 고르면 Nav2로 자율주행하고, 카트에 담은 물건은 카메라가 인식해 휴대폰으로 결제까지 연결합니다.


| 시나리오 | 핵심 기술 |
| --- | --- |
| 사람 추종 | YOLOv8n + BoT-SORT, HSV 히스토그램 Re-ID, YOLOv8n-pose 제스처 |
| 목적지 자율주행 | Nav2 (AMCL + DWB), 사전 SLAM 맵, NavigateToPose Action |
| 무인 인식·결제 | 커스텀 YOLO (best.pt), Flask 웹 UI, QR 결제 |

> **설계 원칙** — 인식 계층(Python)과 제어 계층(C++)을 ROS 2 토픽으로 분리하고, 연산은 워크스테이션에서 수행합니다. Raspberry Pi 4는 센서 데이터를 발행하고, 로봇으로 돌아가는 제어 명령은 `/cmd_vel`입니다.

## 시스템 아키텍처


```text
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
```

## 워크스페이스 구조


```text
src/
├── smart_car_cpp_pkg/                # 실시간 제어 (C++, ament_cmake)
│   ├── include/smart_car_cpp_pkg/
│   │   ├── person_follower.hpp
│   │   ├── go_to_pose.hpp
│   │   └── kalman_filter.hpp
│   ├── src/pi4/
│   │   ├── person_follower.cpp       # 추종 + LiDAR 회피 제어 루프
│   │   ├── go_to_pose.cpp            # Nav2 액션 클라이언트
│   │   └── kalman_filter.cpp         # 4D KF (미적용, '알려진 문제' 참조)
│   ├── config/person_follower.yaml
│   └── launch/person_follower.launch.py
│
└── smart_car_py_pkg/                 # 인식 / GUI / 브릿지 (Python, ament_python)
    ├── smart_car_py_pkg/
    │   ├── pan_tilt_ros2.py          # YOLO 추적 + Re-ID + 제스처 + 팬틸트
    │   ├── servo_udp_bridge.py       # ROS 2 → ESP32 UDP
    │   ├── turtlebot_sound_bridge.py # 알림음 → OpenCR /sound
    │   ├── ros2_cart_bridge.py       # 상품 인식 → Flask REST
    │   ├── cart_gui.py               # Flask 결제 서버
    │   └── gui/
    │       ├── robot_gui.py          # DearPyGUI 관제 노드
    │       ├── map_gui.py            # MapManager — 맵 렌더 / 좌표 변환
    │       ├── button_manager.py     # 버튼 콜백 → /gui_command
    │       ├── gui_node.py           # robot_gui + Flask 동시 실행
    │       └── maps/                 # map.pgm, map.yaml
    ├── pc/yolo_detect_ros2.py        # YOLO 단독 디버그 뷰어
    ├── config/
    │   ├── nav2_params.yaml          # Nav2 전체 스택
    │   └── webcam_params.yaml        # usb_cam ×2 (MJPEG)
    ├── launch/
    │   ├── webcam_launch.py
    │   ├── person_tracking.launch.py
    │   ├── nav2_gui_localization.launch.py
    │   └── cart_system.launch.py
    ├── product_images/               # 상품 썸네일 4종
    └── setup.py
```

## 노드 구성


| 노드 | 언어 | 역할 |
| --- | --- | --- |
| pan_tilt_ros2 | Python | YOLO 추적·Re-ID·제스처·팬 서보 각도 산출 |
| person_follower | C++ | 추종 주행 + LiDAR 장애물 회피 (10Hz) |
| go_to_pose | C++ | Nav2 NavigateToPose 액션 클라이언트 |
| robot_gui | Python | DearPyGUI 관제 화면, 명령 발행 |
| servo_udp_bridge | Python | 서보 명령 → ESP32 UDP |
| turtlebot_sound_bridge | Python | 알림음 → OpenCR /sound |
| ros2_cart_bridge | Python | 상품 YOLO 인식 → Flask REST |
| cart_gui | Python | Flask 결제 서버 (:5000) |

map_gui.py와 button_manager.py는 ROS 노드가 아니라 robot_gui.py가 사용하는 렌더링/콜백 모듈입니다.

### Launch 구성


| Launch | 기동 노드 |
| --- | --- |
| webcam_launch.py | usb_cam ×2 + image_transport republish ×2 + servo_udp_bridge + turtlebot_sound_bridge |
| person_tracking.launch.py | pan_tilt_ros2 + person_follower |
| nav2_gui_localization.launch.py | nav2_bringup + robot_gui + go_to_pose + pan_tilt_ros2 + person_follower |
| cart_system.launch.py | cart_gui + ros2_cart_bridge |

## 기능

### 사람 추종

초기 50초 동안 사용자를 학습한 뒤, 등록된 사람만 따라갑니다.


```text
[t = 0~50s] 학습
   YOLOv8n.track(persist=True, tracker='botsort.yaml')
   → BBox 중앙 ROI  person_img[h*0.2:h*0.8, w*0.25:w*0.75]  → HSV
   → cv2.calcHist([hsv], [0,1], None, [180,256], [0,180, 0,256]) → 정규화
   → master_db 등록,  50초 후 최빈 track_id 를 master_track_id 로 확정
[t > 50s] 추적
   max_sim = max(compareHist(db, hist, HISTCMP_CORREL) for db in master_db)
   마스터 소실 + max_sim > 0.8  → Re-ID, master_track_id 갱신
   그 외                        → Unknown, 무시
```

BoT-SORT가 프레임 간 ID를 유지하고, 가림으로 ID가 끊기면 색상 히스토그램이 복구하는 2단 구조입니다.

거리 측정 — YOLO BBox 크기로 추정하지 않고, 팬 서보가 향한 방향(/pan_tilt/pan_angle)의 LiDAR 값을 직접 읽습니다. 인식은 방향만, 거리는 센서가 담당합니다.

제스처 — YOLOv8n-pose의 17 keypoints를 BBox 기준으로 정규화합니다. 직전 프레임과 공통 가시 키포인트가 6개 이상이고 평균 변위가 0.03 이하면 정지로 보고, 3초 유지 시 추종을 멈춥니다. 무릎(13,14)과 골반(11,12)의 y 차이가 0.08 이하면 "무릎 들기"로 판정해 재개합니다.

### 팬틸트 제어

카메라가 대상을 놓치면 추종이 끊기므로, 팬 서보가 대상을 화면 중앙에 유지합니다.


```text
error_x = BBox 중심 x - 화면 중심 x
   ├── Dead Zone  : |error_x| ≤ 50px 이면 서보 유지
   ├── 비례 보정   : target_pan_angle -= error_x * 0.02
   ├── 클램프      : 0 ~ 180°
   ├── Smoothing  : current += (target - current) * 0.25
   ├── PWM 변환    : np.interp(angle, [0,180], [500,2500]) → μs
   └── 발행        : /servo_pan_cmd (서보)  +  /pan_tilt/pan_angle (주행)
```

Dead Zone과 Smoothing이 없으면 YOLO BBox의 프레임 노이즈가 서보 떨림 → 주행 방향 노이즈로 전파됩니다. 서보 제어 토픽과 주행용 각도 토픽을 분리해, 주행 노드가 서보 구현을 몰라도 되게 했습니다.

### 장애물 회피


```text
   ┌──────────────┐  front ≤ 0.40m  ┌────────────────────────┐
   │  FOLLOWING   │ ───────────────►│  AVOIDANCE_TURN        │
   │              │                 │  선속도 0, 제자리 회전    │
   └──────┬───────┘                 │  좌우 평균 비교 → 넓은 쪽 │
          ▲                         └────────┬───────────────┘
          │ front ≥ 1.30m                    │ pan 각도 정렬 완료
          │ 0.30s 홀드                        ▼
          │                         ┌────────────────────────┐
          └─────────────────────────┤  WALL_FOLLOWING        │
                                    │  P(d=0.65m, kp=0.8)    │
                                    │  선속도 0.08m/s         │
                                    └────────────────────────┘
```


| 섹터 | 중심각 | 연산 | 용도 |
| --- | --- | --- | --- |
| Center | 0.0 | calculateMinDistanceInSector() | 회피 트리거 / 해제 |
| Left | +M_PI/2 | calculateAverageDistanceInSector() | 회피 방향 선택 |
| Right | -M_PI/2 | calculateAverageDistanceInSector() | 회피 방향 선택 |

세 섹터 모두 반각으로 obstacle_front_half_width_rad(0.785rad = ±45°)를 공유합니다.

정면은 최소거리, 좌우는 평균거리를 씁니다. 좌우까지 최소거리로 판정하면 벽 모서리 하나에 과민 반응합니다. 복귀 조건의 0.30초 홀드는 LiDAR 단일 프레임 튐으로 회피↔추종이 진동하는 것을 막습니다.

### 제어 우선순위 및 Safe Stop

controlLoop()는 100ms(10Hz) 타이머로 돌며, 위에서 걸리면 즉시 return합니다.

1. `person_following_enabled_ == false` → `return`
2. `/scan` 또는 pan 각도 미수신 → `publishStop()`
3. 정면 최소거리 ≤ 0.40m → 회피 명령 후 `return`
4. `pose_stop_latched_` → `publishStop()`
5. 검출 후 `lost_timeout_s(0.20s)` 경과 → `publishStop()`
6. LiDAR 거리가 유한하지 않음 → `publishStop()`
7. 정상 → `linear = f(distance)`, `angular = f(pan_angle)`

기본 동작이 정지입니다. Wi-Fi 지연으로 토픽이 밀려도 마지막 명령 속도로 계속 주행하지 않습니다.

### 목적지 자율주행


| 노드 | 라벨 | 좌표 (x, y) |
| --- | --- | --- |
| home | 충전소 | -0.219, 0.052 |
| sunscreen | 선크림 | 2.418, 0.119 |
| wet_tissue | 물티슈 | 2.375, -2.688 |
| stationery | 문구류 | -0.119, -0.874 |
| toilet | 화장실 | 1.598, -1.285 |

GUI 버튼 → /gui_command(CMD_NAV_TO_<destination>) → go_to_pose_cpp가 kDestinations에서 좌표를 찾아 Nav2 navigate_to_pose Action Goal을 전송합니다. 액션 서버 대기 타임아웃은 30초입니다.

모드 배타 제어 — person_follower와 go_to_pose가 모두 /cmd_vel 계통을 제어하므로 동시 활성화되면 안 됩니다. 두 노드가 /gui_command를 함께 구독하고 플래그를 반대로 전환합니다. CMD_SET_MODE_PERSON_FOLLOWING이 오면 go_to_pose가 async_cancel_all_goals()로 Nav2 목표를 즉시 취소하고 잠급니다.

### 무인 인식·결제

ROS 토픽이 아닌 HTTP REST로 연결해, Flask 측이 ROS에 의존하지 않도록 분리했습니다.


```text
/webcam/image_raw/compressed
   → Ros2CartBridge: cv2.imdecode → YOLO(best.pt) 추론
   → confidence(0.6) / cooldown(3.0s) 필터
   → requests.post("http://127.0.0.1:5000/api/add_item")
   → CartManager.add_item()  →  폰 브라우저 폴링(/api/status)
```

동일 상품이 연속 인식되어 수량이 자동 증가하는 것을 막기 위해 cooldown을 두고, 수량 변경은 수동 버튼으로만 처리합니다.


| 엔드포인트 | 용도 |
| --- | --- |
| / | 장바구니 UI |
| /qrcode | 접속용 QR 이미지 |
| /api/status | 장바구니 상태 폴링 |
| /api/add_item | 인식 결과 등록 (ros2_cart_bridge가 호출) |
| /api/update_qty | 수량 +/- |
| /process_payment | 영수증 렌더 + payment_completed 설정 |
| /reset_cart | 카트 초기화 |

### 관제 GUI

robot_gui.py만 rclpy Node이고, map_gui.py·button_manager.py는 DearPyGUI 모듈입니다.


- 라이브 카메라(/webcam2/image_raw/compressed) 임베드, LiDAR 폴라 플롯
- 맵 클릭 → 드래그로 AMCL /initialpose 설정
- TF 기반 로봇 화살표. map→base_link → map→base_footprint → odom→base_link → odom→base_footprint 순으로 fallback 조회하여 AMCL 미기동 시에도 표시
- 배터리 / 상태 오버레이, 목적지 버튼 5개, 추종↔주행 토글
- map_gui.py의 경로 계획(_plan_path_pixels)은 화면 표시용 프리뷰입니다. 실제 주행 경로는 Nav2가 계산합니다.


## 인터페이스

### 카메라 배치


| 토픽 | 구독 노드 |
| --- | --- |
| /webcam2/image_raw/compressed | pan_tilt_ros2 (사람 추종), robot_gui (라이브 뷰) |
| /webcam/image_raw/compressed | ros2_cart_bridge (상품 인식) |

webcam_launch.py가 usb_cam 2대를 각각 /webcam, /webcam2로 remap하고 image_transport republish로 compressed 토픽을 만듭니다.

### 토픽


| 토픽 | 타입 | 방향 |
| --- | --- | --- |
| /person_detection | Float32MultiArray | pan_tilt → follower [center_x, frame_width, confidence] |
| /pan_tilt/pan_angle | Float64 | pan_tilt → follower (deg2rad(angle-90), 중앙 = 0rad) |
| /pose_stationary_detected | Bool | pan_tilt → follower |
| /pose_resume_detected | Bool | pan_tilt → follower |
| /ankle_detected | Bool | pan_tilt → follower |
| /scan | LaserScan | LiDAR → follower, robot_gui |
| /cmd_vel | Twist | follower → OpenCR |
| /servo_pan_cmd, /servo_tilt_cmd | Int32 | pan_tilt → udp_bridge (500–2500μs) |
| /learning_complete_sound_cmd | Int32 | pan_tilt → sound_bridge |
| /sound | turtlebot3_msgs/Sound | sound_bridge → OpenCR |
| /gui_command | String | robot_gui → follower, go_to_pose, pan_tilt |
| /initialpose | PoseWithCovarianceStamped | robot_gui → AMCL |
| /robot_status, /battery_state | String, BatteryState | → robot_gui |

**Action:** `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`), 클라이언트 `go_to_pose_cpp`

### /gui_command 프로토콜


| 명령 | 동작 |
| --- | --- |
| CMD_SET_MODE_PERSON_FOLLOWING | 추종 활성화, Nav2 goal 취소, YOLO 학습 50초 재시작 |
| CMD_SET_MODE_NAVIGATION | 추종 정지, 목적지 명령 수신 대기 |
| `CMD_NAV_TO_{home\|sunscreen\|wet_tissue\|stationery\|toilet}` | 해당 좌표로 Nav2 Action 전송 |

## 빌드 & 실행


```bash
sudo apt update && sudo apt install -y \
    ros-humble-nav2-bringup ros-humble-usb-cam \
    ros-humble-image-transport-plugins ros-humble-turtlebot3-msgs \
    ros-humble-cv-bridge python3-colcon-common-extensions

cd src/smart_car_py_pkg && pip install -r requirements.txt && cd -

colcon build --symlink-install
source install/setup.bash
```


> **YOLO 가중치:** `.gitignore`의 `*.pt` 규칙으로 저장소에는 포함되지 않습니다. `pc/`에 배치하고 다시 빌드하면 `setup.py`가 `share/smart_car_py_pkg/models/`로 설치하며 각 노드가 자동으로 탐색합니다.



```bash
cp yolov8n.pt yolov8n-pose.pt best.pt src/smart_car_py_pkg/pc/
colcon build --symlink-install --packages-select smart_car_py_pkg
```

### 카메라 & 서보 브리지



```bash
ros2 launch smart_car_py_pkg webcam_launch.py \
    esp32_host:=192.168.0.42 esp32_port:=8889
```

### Nav2 + GUI 통합 (메인)



```bash
ros2 launch smart_car_py_pkg nav2_gui_localization.launch.py \
    map:=$(ros2 pkg prefix smart_car_py_pkg)/share/smart_car_py_pkg/gui/maps/map.yaml \
    use_sim_time:=false
```

### 무인 결제



```bash
ros2 launch smart_car_py_pkg cart_system.launch.py \
    image_topic:=/webcam/image_raw/compressed confidence:=0.6 cooldown:=3.0
# → http://<PC IP>:5000
```

## 설정


| 파일 | 내용 |
| --- | --- |
| smart_car_cpp_pkg/config/person_follower.yaml | 추종 거리·속도, 회피 임계, 벽 추종 게인 |
| smart_car_py_pkg/config/nav2_params.yaml | AMCL(DifferentialMotionModel), FollowPath(DWB), Costmap |
| smart_car_py_pkg/config/webcam_params.yaml | usb_cam ×2, MJPEG, 해상도 |

`person_follower`는 파라미터 콜백(`onParameterUpdate`)을 구현해 재빌드 없이 실시간 변경할 수 있습니다.


```bash
ros2 param set /person_follower follow_distance_m 1.50
ros2 param set /person_follower obstacle_trigger_distance_m 0.35
ros2 run rqt_reconfigure rqt_reconfigure
```

자주 만지게 되는 값들입니다. 전체 목록은 yaml을 참조하세요.


| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| stop_distance_m / follow_distance_m / far_distance_m | 1.00 / 1.30 / 1.80 m | 정지 / 추종 / 가속 거리 |
| normal_linear_velocity / fast_linear_velocity | 0.10 / 0.15 m/s | 선속도 |
| max_angular_velocity / body_turn_kp | 0.22 rad/s / 0.30 | 회전 |
| lost_timeout_s | 0.20 s | 검출 유실 판정 (Safe Stop) |
| min_detection_confidence | 0.50 | YOLO 추종 최소 신뢰도 |
| obstacle_trigger_distance_m | 0.40 m | 회피 트리거 |
| front_clear_distance_m / front_clear_hold_s | 1.30 m / 0.30 s | 회피 해제 |
| wall_follow_target_distance_m / wall_follow_kp | 0.65 m / 0.80 | 벽 추종 |

**`pan_tilt_ros2` 제스처 임계값:** `pose_stationary_motion_threshold=0.03`, `pose_resume_knee_hip_tolerance=0.08`, `pose_stationary_min_keypoints=6`

## 트러블슈팅


| 증상 | 원인 / 해결 |
| --- | --- |
| 카메라 영상이 안 나옴 | ls /dev/video* → webcam_params.yaml의 video_device 조정. v4l2-ctl -d /dev/videoN --list-formats-ext로 MJPEG 지원 확인 |
| 로봇이 아예 안 움직임 | CMD_SET_MODE_PERSON_FOLLOWING 전송 여부 확인. person_following_enabled_가 false면 controlLoop이 즉시 return |
| Nav2 goal 거부됨 | CMD_SET_MODE_NAVIGATION 선행 필요 |
| 추종 시 좌우 헤맴 | body_turn_kp 하향(0.3 → 0.2), aligned_angle_threshold_rad(0.08) 상향 |
| 학습 후 'Master' 미인식 | 50초간 한 명만 정면 유지. min_detection_confidence 0.5 → 0.4 |
| 회피 후 복귀 안 됨 | front_clear_distance_m(1.30), front_clear_hold_s(0.30) 확인. /scan 수신 점검 |
| 회피/추종이 진동 | front_clear_hold_s 또는 min_avoidance_active_s(0.80) 상향 |
| 서보가 미세하게 떨림 | Dead Zone(50px) 상향, Smoothing 계수(0.25) 하향 |
| ESP32 서보 무응답 | servo_udp_bridge 로그에서 UDP 대상 IP/포트 확인. ESP32가 8889에서 수신 대기 중인지 확인 |
| 상품이 장바구니에 안 담김 | cart_api_url과 cart_gui 기동 여부 확인 |
| turtlebot3_msgs/Sound 에러 | sudo apt install ros-humble-turtlebot3-msgs |
| 제스처 후 추종 시작이 몇 초 늦음 | 추적 모델이 yolov8s면 4초 이상 지연. yolov8n 사용 — 아래 참조 |

### 추적 모델 스케일과 제스처 반응 지연

추적 정밀도를 높이려 yolov8n → yolov8s로 올린 적이 있습니다. 추종 자체는 동작했지만 제스처 후 추종 시작까지 4초 이상 걸렸고, yolov8n에서는 즉시 반응했습니다.

관측된 조건은 다음과 같습니다.


- 추론은 워크스테이션 GPU (device='cuda:0', imgsz=320)
- 처리 주기 process_period_sec = 0.08s (12.5Hz)
- 이미지 QoS는 BEST_EFFORT / KEEP_LAST / depth=1 — 구독 측 적체 없음
- process_latest_frame() 한 번에 pose 모델과 추적 모델이 순차 추론

정상 상태 추론 시간으로는 80ms 예산을 넘길 이유가 없어 4초가 설명되지 않습니다. 모델 최초 로드 시점(get_model()은 첫 프레임에서 lazy load)의 CUDA 워밍업과 카메라 전송 지연을 후보로 두었으나 계측까지 완료하지 못했고, 반응성이 확보되는 yolov8n을 유지했습니다.

재현·계측이 필요한 항목입니다. 추론 소요 시간과 프레임 도착 간격을 각각 로깅하면 지연이 추론·전송·최초 로드 중 어디에서 발생하는지 분리할 수 있습니다.

## 알려진 문제


- kalman_filter.cpp는 빌드에 포함되지만 호출되지 않습니다. 가림 구간 궤적 유지용 4D(x, y, vx, vy) 필터로 작성했으나, BoT-SORT가 내부적으로 칼만 필터를 사용하므로 이중 적용할 이유가 없어 연결하지 않았습니다.
- webcam_params.yaml의 주석과 실제 코드의 카메라 배치가 어긋나 있습니다. 주석은 webcam=사람 추종으로 설명하지만, pan_tilt_ros2.py는 /webcam2를 구독합니다. 실제 배선 기준으로 주석 정리가 필요합니다.
- 파이프라인 구간별 지연 계측 수단이 없습니다. 카메라 발행 → 전송 → 추론 → /cmd_vel까지의 latency를 측정할 방법이 없어 성능 문제 시 원인 구간을 특정하기 어렵습니다. 위 yolov8s 4초 지연을 규명하지 못한 이유이기도 합니다.
- 모델이 lazy load됩니다. 첫 추론에 모델 로드와 CUDA 워밍업 비용이 함께 실립니다. 노드 기동 시 더미 추론으로 워밍업하면 첫 검출 지연을 없앨 수 있습니다. _resolve_model_path()가 파일을 못 찾으면 이름만 반환해 ultralytics가 자동 다운로드하는 점도 정리 대상입니다.
- 목적지 좌표가 go_to_pose.cpp에 하드코딩되어 있어 맵 변경 시 재빌드가 필요합니다. YAML 외부화 대상입니다.
- 모드/상태를 열거형이 아닌 불리언 플래그로 관리합니다. /cmd_vel을 다투는 노드가 둘뿐이라 문제되지 않았으나, 상태가 늘어나면 명시적 FSM으로 정리해야 합니다.
- 벽 추종은 한쪽 면만 참조합니다. 양측이 모두 좁은 통로는 검증하지 않았습니다.
- requirements.txt의 mysql-connector-python은 현재 사용하지 않습니다.

## 하드웨어 / 소프트웨어 환경


| 항목 | 사양 |
| --- | --- |
| 로봇 베이스 | TurtleBot3 Waffle Pi |
| 컨트롤러 | OpenCR 1.0 (Dynamixel XM430-W210) |
| SBC | Raspberry Pi 4 (4GB, Ubuntu 22.04 Server) |
| LiDAR | Robotis LDS-02 |
| 카메라 | USB Webcam ×2 (MJPEG 320×240, 15fps) |
| 팬틸트 | ESP32 + Servo ×2 (μs 단위 제어) |
| 워크스테이션 | Ubuntu 22.04 + NVIDIA GPU (CUDA 추론) |
| ROS 2 | Humble Hawksbill |
| AI 모델 | YOLOv8n (추적) / YOLOv8n-pose (제스처) / best.pt (상품 4종) |

## 팀 구성


| 역할 | 담당자 | 주요 기여 |
| --- | --- | --- |
| 객체 탐지 / Re-ID | 이주석 | YOLOv8n + BoT-SORT 추적, Arduino 팬틸트, Git 형상관리 |
| 추종 제어 / Nav2 | 한수창 | 객체 추종 주행 로직, 경로 계획, 팬틸트 서보 제어 |
| 추종 제어 / 팬틸트 | 신종현 | 객체 추종 주행 로직, Arduino 팬틸트 |
| 하드웨어 / DL 학습 | 성대현 | 하드웨어 통합, 상품 인식 YOLO 학습, 웹페이지 연동 |
| Web / GUI | 송훈정 | 스마트폰 장바구니 웹, 관제 GUI, 라벨링 |


> 로보테크 AI 자율주행 로봇 개발자 과정(2026.05) 1조의 14일 종합 프로젝트 산출물입니다.

## 개발 히스토리


| 단계 | 기간 | 내용 |
| --- | --- | --- |
| 기획·설계 | Day 1–2 | 요구사항 분석, 시스템 아키텍처 설계, 기술 스택 선정 |
| 환경 구성 | Day 3–5 | TurtleBot3 ↔ Pi4 ROS2 연동, 웹캠/LiDAR 드라이버 설치 |
| AI 모듈 개발 | Day 6–8 | YOLOv8 + BoT-SORT 추종 모듈, Re-ID, 팬틸트 연동 |
| 자율주행 구현 | Day 9–11 | SLAM 맵 구축, Nav2 자율주행, 장애물 회피 검증 |
| UI 개발 / 통합 | Day 12–14 | 결제 UI, REST 연동, 시연·버그 픽스 |

## 라이선스

본 저장소는 교육 목적으로 작성되었습니다. 외부 사용 시 팀에 사전 문의 부탁드립니다.

## 참고 문헌


- ROBOTIS TurtleBot3 e-Manual
- Nav2 Documentation (Humble)
- Ultralytics YOLOv8
- BoT-SORT: Robust Associations Multi-Pedestrian Tracking
- ROS 2 Humble Hawksbill
