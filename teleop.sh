#!/usr/bin/env bash
# 엑스박스 컨트롤러(USB 유선)로 핑키 조종 — 호스트에서 실행.
# RT = 가속, LT = 브레이크/후진, 오른쪽 스틱 = 좌우 조향. orchestrator 주행(NAVIGATING) 중엔 쓰지 말 것.
# -u 금지: ROS setup.bash가 미정의 변수를 참조해 죽는다
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run joy joy_node &
JOY_PID=$!
trap 'kill "$JOY_PID" 2>/dev/null' EXIT
ros2 run gukbapjipmaknaeadeul teleop_racing
