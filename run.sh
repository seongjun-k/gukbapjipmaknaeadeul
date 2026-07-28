#!/usr/bin/env bash
# gukbapjipmaknaeadeul orchestrator(웹 UI 포함) 실행 — 호스트, 워크스페이스 루트에서
# -u 금지: ROS setup.bash가 미정의 변수를 참조해 죽는다
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] || colcon build --symlink-install
source install/setup.bash
exec ros2 run gukbapjipmaknaeadeul orchestrator
