# ShelfBot

자율주행(pinky pro) + 모방학습(soarm101, ACT) 기반 무인 상품 진열 시스템.

- 계획: [구현계획서.md](구현계획서.md) / 기획서: ShelfBot_프로젝트_기획서.docx
- 구조: colcon 워크스페이스. `src/shelfbot/`(ROS2 패키지: orchestrator·도킹·웹 UI·config) + `arm/`(LeRobot, 별도 venv — 워크스페이스 밖)
