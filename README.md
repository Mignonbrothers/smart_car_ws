# 🛒 Smart Cart — TurtleBot3 기반 자율주행 쇼핑 로봇

> **AI · 자율주행 · IoT 융합형 차세대 지능형 리테일 플랫폼**
>
> YOLOv8n + BoT-SORT 기반 사용자 추종, Nav2 자율주행, ROSbridge 연동 무인 결제 시스템.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blueviolet)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![C++](https://img.shields.io/badge/C++-17-00599C)
![YOLOv8](https://img.shields.io/badge/YOLOv8-n%2Fs%2Fpose-success)
![TurtleBot3](https://img.shields.io/badge/Robot-TurtleBot3%20Waffle%20Pi-red)
![License](https://img.shields.io/badge/License-Educational-lightgrey)

---

## 프로젝트 소개

TurtleBot3 Waffle Pi에 팬틸트 웹캠, 2D LiDAR, ESP32 서보 제어기를 결합한 자율주행 쇼핑 카트입니다.

평소에는 등록된 사용자를 따라다니고, 터치 패널에서 상품 위치나 편의시설을 선택하면 Nav2로 목적지까지 이동합니다. 카트에 담은 상품은 별도 카메라가 인식하며, 휴대폰 웹 화면에서 수량 확인과 결제를 진행할 수 있습니다.

이 프로젝트는 로보테크 AI 자율주행 로봇 개발자 과정에서 5명이 14일 동안 진행한 팀 프로젝트입니다. AI 연산과 GUI는 Ubuntu 워크스테이션에서 처리하고, Raspberry Pi 4는 TurtleBot의 센서와 구동부를 담당하도록 구성했습니다.

## 주요 기능

| 기능 | 구현 내용 |
| --- | --- |
| 사람 추종 | YOLOv8n + BoT-SORT, HSV 히스토그램 Re-ID |
| 제스처 인식 | YOLOv8n-pose를 이용한 정지·재개 동작 인식 |
| 목적지 이동 | SLAM 맵, AMCL, Nav2 `NavigateToPose` |
| 장애물 회피 | 2D LiDAR 기반 정지·회전·벽 추종 |
| 팬틸트 카메라 | ESP32와 2축 서보를 이용한 사용자 화면 중앙 추적 |
| 상품 인식 | 커스텀 YOLO 모델로 4종 상품 인식 |
| 무인 결제 | Flask 장바구니와 QR 접속 기반 모바일 결제 화면 |

## 시스템 구성

```text
Ubuntu 22.04 Workstation
├── pan_tilt_ros2.py        사용자 탐지, Re-ID, 제스처, 팬틸트 제어
├── person_follower         C++ 추종 주행 및 LiDAR 장애물 회피
├── go_to_pose              Nav2 목적지 전송
├── robot_gui.py            DearPyGUI 관제 화면
├── ros2_cart_bridge.py     상품 인식 결과를 Flask로 전달
└── cart_gui.py             모바일 장바구니 및 결제 서버
              │
              │ ROS 2 DDS / Wi-Fi
              ▼
TurtleBot3 Waffle Pi
├── Raspberry Pi 4
├── OpenCR + Dynamixel
├── LDS-02 LiDAR
└── USB Webcam ×2

ESP32 + Pan/Tilt Servo
└── UDP 8889로 팬·틸트 PWM 명령 수신
```

Python 노드는 카메라 인식과 GUI를 담당하고, C++ 노드는 10Hz 제어 루프에서 실제 주행 명령을 계산합니다. 최종 속도 명령은 `/cmd_vel`을 통해 TurtleBot으로 전달됩니다.

## 워크스페이스

```text
src/
├── smart_car_cpp_pkg/
│   ├── include/smart_car_cpp_pkg/
│   │   ├── person_follower.hpp
│   │   ├── go_to_pose.hpp
│   │   └── kalman_filter.hpp
│   ├── src/pi4/
│   │   ├── person_follower.cpp
│   │   ├── go_to_pose.cpp
│   │   └── kalman_filter.cpp
│   ├── config/person_follower.yaml
│   └── launch/person_follower.launch.py
│
└── smart_car_py_pkg/
    ├── smart_car_py_pkg/
    │   ├── pan_tilt_ros2.py
    │   ├── servo_udp_bridge.py
    │   ├── turtlebot_sound_bridge.py
    │   ├── ros2_cart_bridge.py
    │   ├── cart_gui.py
    │   └── gui/
    │       ├── robot_gui.py
    │       ├── map_gui.py
    │       └── button_manager.py
    ├── config/
    │   ├── nav2_params.yaml
    │   └── webcam_params.yaml
    ├── launch/
    ├── product_images/
    └── setup.py
```

## 사람 추종

사람 추종은 인식과 주행 제어를 분리해서 구현했습니다.

`pan_tilt_ros2.py`가 카메라 영상에서 사람을 찾고, 화면 중심 좌표와 신뢰도를 `/person_detection`으로 발행합니다. `person_follower.cpp`는 이 값과 팬 각도, LiDAR 거리를 함께 사용해 선속도와 각속도를 계산합니다.

처음 50초 동안 사람의 상체 영역에서 HSV 히스토그램을 수집하고, 가장 자주 검출된 Track ID를 사용자로 등록합니다. BoT-SORT의 ID가 가림 등으로 끊긴 경우에는 등록된 히스토그램과 현재 사람의 히스토그램을 비교해 사용자를 다시 찾습니다.

거리는 YOLO Bounding Box 크기로 추정하지 않고, 팬 서보가 바라보는 방향의 LiDAR 값을 사용했습니다. 카메라는 방향을 찾고 LiDAR는 거리를 측정하는 방식입니다.

### 정지와 재개

YOLOv8n-pose의 키포인트 움직임이 일정 시간 작으면 사용자가 정지한 것으로 판단합니다. 정지 상태가 3초간 유지되면 로봇도 멈춥니다. 이후 사용자가 무릎을 들어 올리면 추종을 다시 시작합니다.

### 팬틸트

사람의 중심이 화면 중앙에서 벗어나면 팬 서보 각도를 보정합니다. 작은 검출 오차에 서보가 계속 반응하지 않도록 50px Dead Zone을 두었고, 목표 각도에는 Smoothing을 적용했습니다.

```text
화면 중심 오차 계산
→ Dead Zone 적용
→ 비례 보정
→ 0~180° 제한
→ Smoothing
→ 500~2500μs PWM 변환
→ ESP32로 전송
```

## 장애물 회피

정면 0.4m 이내에서 장애물이 감지되면 먼저 정지한 뒤, LiDAR 좌우 평균거리를 비교해 공간이 넓은 방향으로 회전합니다. 팬 각도가 다시 사용자 방향과 맞으면 벽과 약 0.65m 간격을 유지하며 전진합니다.

정면 공간이 1.3m 이상 확보된 상태가 0.3초 동안 유지되면 일반 추종으로 돌아갑니다. 한 프레임의 LiDAR 노이즈 때문에 추종과 회피가 반복 전환되지 않도록 복귀 조건에 유지 시간을 두었습니다.

카메라 검출이나 LiDAR 데이터가 일정 시간 들어오지 않으면 이전 속도를 유지하지 않고 정지하도록 구현했습니다.

## 목적지 자율주행

| 목적지 | 좌표 `(x, y)` |
| --- | --- |
| 충전소 | `(-0.219, 0.052)` |
| 선크림 | `(2.418, 0.119)` |
| 물티슈 | `(2.375, -2.688)` |
| 문구류 | `(-0.119, -0.874)` |
| 화장실 | `(1.598, -1.285)` |

GUI에서 목적지를 선택하면 `/gui_command`로 명령을 보냅니다. `go_to_pose` 노드는 목적지 좌표를 찾아 Nav2 `NavigateToPose` Action Goal을 전송합니다.

사람 추종과 Nav2가 동시에 로봇을 제어하지 않도록 두 모드를 분리했습니다. 추종 모드로 전환하면 실행 중인 Nav2 Goal을 취소하고, 자율주행 모드에서는 사람 추종 속도 명령을 중지합니다.

## 상품 인식과 결제

카트 상단 카메라 영상은 `ros2_cart_bridge.py`에서 커스텀 YOLO 모델로 처리합니다. 상품이 인식되면 Flask 서버의 `/api/add_item`으로 결과를 보내고, 휴대폰 장바구니 화면에 반영합니다.

같은 상품이 연속으로 등록되지 않도록 신뢰도 0.6과 3초 Cooldown을 적용했습니다. 장바구니에서는 사용자가 직접 수량을 조정하고 결제를 진행할 수 있습니다.

```text
/webcam/image_raw/compressed
→ YOLO(best.pt)
→ confidence / cooldown 확인
→ Flask REST API
→ 모바일 장바구니
```

## 관제 GUI

DearPyGUI 기반으로 카메라 영상, LiDAR 스캔, 로봇 위치, 배터리와 현재 상태를 한 화면에서 확인할 수 있도록 만들었습니다.

- 맵 클릭과 드래그로 AMCL 초기 위치 설정
- 목적지 버튼 5개
- 사람 추종·자율주행 모드 전환
- 카메라 영상과 LiDAR 폴라 플롯
- TF 기반 로봇 위치 표시

GUI에 표시되는 경로는 화면 확인용 프리뷰이며, 실제 주행 경로는 Nav2가 계산합니다.

## 주요 ROS 2 인터페이스

| 토픽 | 용도 |
| --- | --- |
| `/person_detection` | 사람 중심 좌표와 신뢰도 |
| `/pan_tilt/pan_angle` | 주행 제어에 사용하는 팬 각도 |
| `/pose_stationary_detected` | 사용자 정지 동작 |
| `/pose_resume_detected` | 사용자 재개 동작 |
| `/scan` | LiDAR 데이터 |
| `/cmd_vel` | TurtleBot 속도 명령 |
| `/servo_pan_cmd`, `/servo_tilt_cmd` | ESP32 서보 PWM 명령 |
| `/gui_command` | 모드 변경과 목적지 선택 |
| `/initialpose` | AMCL 초기 위치 |

사람 추종 카메라는 `/webcam2/image_raw/compressed`, 상품 인식 카메라는 `/webcam/image_raw/compressed`를 사용합니다.

## 빌드 및 실행

ROS 2 Humble과 TurtleBot3, Nav2가 설치된 Ubuntu 22.04 환경을 기준으로 합니다.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-usb-cam \
  ros-humble-image-transport-plugins \
  ros-humble-turtlebot3-msgs \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions

cd src/smart_car_py_pkg
pip install -r requirements.txt
cd ../..

colcon build --symlink-install
source install/setup.bash
```

YOLO 가중치는 저장소에 포함되어 있지 않습니다. 아래 파일을 `src/smart_car_py_pkg/pc/`에 넣고 Python 패키지를 다시 빌드해야 합니다.

```text
yolov8n.pt
yolov8n-pose.pt
best.pt
```

### Nav2와 GUI

```bash
ros2 launch smart_car_py_pkg nav2_gui_localization.launch.py \
  map:=$(ros2 pkg prefix smart_car_py_pkg)/share/smart_car_py_pkg/gui/maps/map.yaml \
  use_sim_time:=false
```

### 카메라와 팬틸트

```bash
ros2 launch smart_car_py_pkg webcam_launch.py \
  esp32_host:=192.168.0.42 \
  esp32_port:=8889
```

### 장바구니와 결제

```bash
ros2 launch smart_car_py_pkg cart_system.launch.py \
  image_topic:=/webcam/image_raw/compressed \
  confidence:=0.6 \
  cooldown:=3.0
```

실행 후 휴대폰에서 `http://<PC IP>:5000`으로 접속합니다.

## 개발 환경

| 항목 | 사양 |
| --- | --- |
| 로봇 | TurtleBot3 Waffle Pi |
| SBC | Raspberry Pi 4 4GB |
| 컨트롤러 | OpenCR 1.0 |
| LiDAR | ROBOTIS LDS-02 |
| 카메라 | USB Webcam ×2 |
| 팬틸트 | ESP32 + Servo ×2 |
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| AI 모델 | YOLOv8n, YOLOv8n-pose, custom `best.pt` |

## 팀

| 담당 | 이름 | 작업 내용 |
| --- | --- | --- |
| 객체 탐지 / Re-ID | 이주석 | YOLOv8n + BoT-SORT, 팬틸트, Git 관리 |
| 추종 제어 / Nav2 | 한수창 | 추종 주행, 경로 계획, 팬틸트 서보 제어 |
| 추종 제어 / 팬틸트 | 신종현 | 추종 주행, Arduino 팬틸트 |
| 하드웨어 / 상품 학습 | 성대현 | 하드웨어 통합, 상품 YOLO 학습, 웹 연동 |
| Web / GUI | 송훈정 | 모바일 장바구니, 관제 GUI, 데이터 라벨링 |

## 참고

- [ROBOTIS TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Navigation2 Documentation](https://docs.nav2.org/)
- [Ultralytics YOLO](https://docs.ultralytics.com/)
- [BoT-SORT](https://arxiv.org/abs/2206.14651)

## License

교육 목적으로 제작한 프로젝트입니다. 외부 사용 시 팀에 문의해 주세요.
