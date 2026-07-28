# gukbapjipmaknaeadeul

자율주행(pinky pro) + 모방학습(soarm101, ACT) 기반 무인 상품 진열 시스템.
웹 UI에서 ▶시작 → pinky가 Nav2로 진열대 앞 주행 → 아루코 마커 정밀 도킹 → soarm101이 ACT 정책으로 상품을 지정 칸에 진열.

- 설계 SSoT: [docs/구현계획서.md](docs/구현계획서.md) / 기획서: docs/ShelfBot_프로젝트_기획서.docx
- 환경: Ubuntu 24.04, ROS2 Jazzy, Python 3.12 / 팀: 이기문, 강성준

## 구조

```
src/gukbapjipmaknaeadeul/            # ROS2 ament_python 패키지 (시스템 파이썬, rclpy)
├── gukbapjipmaknaeadeul/
│   ├── orchestrator_node.py  # 상태머신 노드 + uvicorn 스레드 (단일 프로세스)
│   ├── machine.py            # 순수 파이썬 상태머신 (READY→NAVIGATING→DOCKING→PLACING→DONE/FAILED)
│   ├── nav_client.py         # Nav2 NavigateToPose 래퍼
│   ├── aruco_docking.py      # 탑뷰 캠 아루코 → cmd_vel P제어 정밀 도킹
│   ├── calibrate_camera.py   # 탑뷰 캠 내부 파라미터 캘리브레이션
│   ├── frame_hub.py          # USB 캠 단일 리더 스레드 (경합 방지)
│   └── web_server.py         # FastAPI: REST + WebSocket + MJPEG
├── config/booth.yaml         # goal pose·도킹 게인/임계값·칸 매핑 (실측값 TBD)
├── config/cameras.yaml       # 캠 장치 경로 (TBD)
└── web/static/index.html     # 시연 UI 한 장 (4분할 + SLAM 지도)
arm/                     # LeRobot (별도 venv, torch — 프로세스 경계로만 통신)
├── run_policy.py        # ACT 체크포인트 → 1회 진열 실행 → 종료코드
├── record.sh            # 텔레옵 녹화 래퍼
└── train_remote.sh      # 5090 서버 원격 학습 (rsync + ssh)
```

## 빌드·테스트

```bash
# 워크스페이스 루트에서
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
pytest src/gukbapjipmaknaeadeul/test/ -q
```

의존: `python3-fastapi`, `python3-uvicorn`, `python3-httpx`(테스트), OpenCV(ros-jazzy 기본).

## 실행

```bash
ros2 launch gukbapjipmaknaeadeul shelfbot.launch.py   # orchestrator + 웹 (기본 :8000)
```

실기 구동 전 필수: 부스 세팅(구현계획서 §3) 후 `booth.yaml`의 goal_pose·dock target_pose·캘리브 파일·pinky 토픽명, `cameras.yaml` 장치 경로 실측 기입.
