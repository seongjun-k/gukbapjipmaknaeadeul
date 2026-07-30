# gukbapjipmaknaeadeul

자율주행(pinky pro) + 모방학습(soarm101, ACT) 기반 무인 상품 진열 시스템.
웹 UI에서 ▶시작 → pinky가 Nav2로 진열대 앞 주행 → soarm101이 ACT 정책으로 상품을 지정 칸에 진열.
Nav2(map_server·AMCL·planner)는 노트북에서 실행하고, 핑키 온보드는 pinky_bringup(모터·라이다·odom/tf)만 돈다.

- 설계 SSoT: [docs/구현계획서.md](docs/구현계획서.md) / 기획서: docs/ShelfBot_프로젝트_기획서.docx
- 환경: Ubuntu 24.04, ROS2 Jazzy, Python 3.12 / 팀: 이기문, 강성준, 이성관, 함정현, 석지훈

## 구조

```
src/gukbapjipmaknaeadeul/         # ROS2 ament_python 패키지 (시스템 파이썬, rclpy)
├── gukbapjipmaknaeadeul/
│   ├── orchestrator_node.py      # 상태머신 노드 + uvicorn 스레드 (단일 프로세스)
│   ├── machine.py                # 순수 파이썬 상태머신 (READY→NAVIGATING→PLACING→DONE/FAILED)
│   ├── nav_client.py             # Nav2 NavigateToPose 래퍼
│   ├── teleop_racing.py          # 엑스박스 컨트롤러 텔레옵 (RT 가속·LT 브레이크·좌스틱 조향)
│   ├── frame_hub.py              # USB 캠 단일 리더 스레드 (경합 방지)
│   └── web_server.py             # FastAPI: REST + WebSocket + MJPEG + Nav2 파라미터 패널
├── launch/gukbapjipmaknaeadeul.launch.py  # Nav2 bringup(노트북) + orchestrator
├── config/booth.yaml             # goal pose·칸 매핑·웹 포트 (실측값 TBD)
├── config/cameras.yaml           # 캠 장치 경로 (TBD)
├── config/nav2_params.yaml       # Nav2 파라미터 (pinky_navigation 기반)
├── config/gukbab_map.{pgm,yaml}  # SLAM 지도
└── web/static/index.html         # 시연 UI 한 장 (4분할 + SLAM 지도 + 코스트맵 오버레이)
arm/                     # LeRobot (별도 venv, torch — 프로세스 경계로만 통신)
├── run_policy_wrapper.sh # lerobot-rollout 호출 래퍼 (booth.yaml placing.cmd)
├── mjpeg_camera.py      # 웹 MJPEG 스트림을 lerobot 카메라로 쓰는 어댑터
├── record.sh            # 텔레옵 녹화 래퍼
└── train_remote.sh      # 5090 서버 원격 학습 (rsync + ssh)
tools/pinky_cam_stream.py # 핑키(라즈베리파이)에서 실행 — CSI 캠 picamera2 MJPEG 스트리머 (:8081)
```

## 빌드·테스트

```bash
# 호스트, 워크스페이스 루트(~/gukbapjipmaknaeadeul)에서
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
pytest src/gukbapjipmaknaeadeul/test/ -q
```

의존: `python3-fastapi`, `python3-uvicorn`, `python3-httpx`(테스트), OpenCV(ros-jazzy 기본), `nav2_bringup`.

## 실행

```bash
./run.sh      # orchestrator + 웹 UI (기본 :8000) — Nav2는 launch로 별도 실행
ros2 launch gukbapjipmaknaeadeul gukbapjipmaknaeadeul.launch.py  # Nav2 bringup + orchestrator 일괄 실행
./teleop.sh   # 엑스박스 컨트롤러 텔레옵 (orchestrator 주행 중 사용 금지)
```

핑키 캠: 핑키에서 `tools/pinky_cam_stream.py` 실행 → 웹 UI가 `http://192.168.4.1:8081/stream` 직접 표시.

실기 구동 전 필수: 부스 세팅(구현계획서 §3) 후 `booth.yaml`의 goal_pose·pinky 토픽명, `cameras.yaml` 장치 경로 실측 기입.
