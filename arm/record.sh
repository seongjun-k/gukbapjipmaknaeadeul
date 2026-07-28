#!/usr/bin/env bash
# lerobot 데이터 녹화 래퍼 (lerobot venv에서 실행, 호스트: soarm101 연결된 로컬 머신)
# 사용법: ./record.sh <product_label> <num_episodes> [extra lerobot-record args...]
set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "사용법: $0 <product_label> <num_episodes> [extra args...]" >&2
    exit 1
fi

PRODUCT="$1"
NUM_EPISODES="$2"
shift 2

DATE_TAG="$(date +%Y%m%d)"
DATASET_NAME="gukbapjipmaknaeadeul_v1_${DATE_TAG}_${PRODUCT}"

# TODO: 실제 로봇/캠 설정 이름은 lerobot 설치 버전의 CLI 옵션에 맞춰 확정
lerobot-record \
    --robot.type=soarm101 \
    --dataset.repo_id="local/${DATASET_NAME}" \
    --dataset.num_episodes="${NUM_EPISODES}" \
    --dataset.single_task="${PRODUCT} 진열" \
    "$@"
