#!/usr/bin/env bash
# lerobot ACT 정책 실행 래퍼 — booth.yaml placing.cmd 가 서브프로세스로 호출.
# lerobot-rollout(base 전략)이 정책 로드·로봇 연결·정지 신호(SIGTERM/INT) 시
# 초기 자세 복귀까지 처리한다 (rollout/strategies/core.py return_to_initial_position=True).
# 종료코드: 0 = duration 완주(마지막에 초기 자세 복귀), 그 외 = 실패.
set -euo pipefail
cd "$(dirname "$0")"

# TBD: 학습 완료 후 실측 확정 (환경변수로 덮어쓰기 가능)
CHECKPOINT="${GUKBAPJIPMAKNAEADEUL_CKPT:-./checkpoints/act_gukbapjipmaknaeadeul/pretrained_model}"
PORT="${GUKBAPJIPMAKNAEADEUL_ARM_PORT:-/dev/ttyACM0}"   # TBD: soarm101 시리얼 포트
TASK="${GUKBAPJIPMAKNAEADEUL_TASK:-진열}"
DURATION="${GUKBAPJIPMAKNAEADEUL_DURATION:-60}"          # booth.yaml placing.timeout_sec(90)보다 짧아야 kill 경로를 안 탐

# 캠은 장치를 직접 열지 않는다(frame hub와 USB 경합, 구현계획서 §6.6/§7.2)
# — orchestrator 웹 서버의 MJPEG 스트림을 mjpeg_camera.py의 'mjpeg' 타입으로 공급.
# width/height는 cameras.yaml, fps는 web_server._STREAM_FPS(15)와 일치시킬 것.
CAMERAS='{ hand: {type: mjpeg, index_or_path: "http://127.0.0.1:8000/stream/hand", width: 640, height: 480, fps: 15}}'

# mjpeg 타입은 CLI 파싱 전에 등록돼야 하므로 임포트 후 rollout main 호출
exec .venv/bin/python -c "import mjpeg_camera; from lerobot.scripts.lerobot_rollout import main; main()" \
    --strategy.type=base \
    --policy.path="${CHECKPOINT}" \
    --robot.type=so101_follower \
    --robot.port="${PORT}" \
    --robot.cameras="${CAMERAS}" \
    --task="${TASK}" \
    --duration="${DURATION}"
