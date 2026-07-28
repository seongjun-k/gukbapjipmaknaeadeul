#!/usr/bin/env bash
# 엑스박스 컨트롤러로 핑키 조종 — 호스트에서 실행, 컨트롤러 USB/블루투스 연결 필요.
# LT를 누른 동안만 /cmd_vel 퍼블리시(데드맨), RT = 터보. orchestrator 주행(NAVIGATING) 중엔 쓰지 말 것.
# -u 금지: ROS setup.bash가 미정의 변수를 참조해 죽는다
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
exec ros2 launch teleop_twist_joy teleop-launch.py \
    config_filepath:="$(pwd)/src/gukbapjipmaknaeadeul/config/teleop_xbox.yaml"
