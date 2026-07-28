#!/usr/bin/env bash
# 로컬에서 녹화한 데이터셋을 원격 5090 서버로 업로드 → ACT 학습 → 체크포인트 회수
# 사용법: ./train_remote.sh <local_dataset_dir> <run_name>
set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "사용법: $0 <local_dataset_dir> <run_name>" >&2
    exit 1
fi

LOCAL_DATASET_DIR="$1"
RUN_NAME="$2"

# TODO: 실제 서버 정보로 교체
REMOTE_HOST="TBD"          # 예: user@5090-server
REMOTE_DATA_DIR="TBD"      # 예: ~/gukbapjipmaknaeadeul_data
REMOTE_CKPT_DIR="TBD"      # 예: ~/gukbapjipmaknaeadeul_ckpt
LOCAL_CKPT_DIR="${LOCAL_CKPT_DIR:-./checkpoints/${RUN_NAME}}"

echo "[train_remote] torch/cuda 환경 확인 중..."
ssh "${REMOTE_HOST}" "python3 -c \"import torch; assert torch.__version__ >= '2.7', torch.__version__; assert torch.cuda.is_available(); assert '128' in (torch.version.cuda or ''), torch.version.cuda\"" \
    || { echo "[train_remote] 서버 torch 2.7+/cu128 확인 실패 — 중단" >&2; exit 1; }

echo "[train_remote] 데이터셋 업로드 중..."
rsync -avz --progress "${LOCAL_DATASET_DIR}/" "${REMOTE_HOST}:${REMOTE_DATA_DIR}/${RUN_NAME}/"

echo "[train_remote] 원격 학습 시작..."
# TODO: 실제 lerobot train 옵션(policy.type=act 등) 확정
ssh "${REMOTE_HOST}" "cd ${REMOTE_DATA_DIR} && lerobot train \
    --policy.type=act \
    --dataset.repo_id=local/${RUN_NAME} \
    --output_dir=${REMOTE_CKPT_DIR}/${RUN_NAME}"

echo "[train_remote] 체크포인트 회수 중..."
mkdir -p "${LOCAL_CKPT_DIR}"
rsync -avz --progress "${REMOTE_HOST}:${REMOTE_CKPT_DIR}/${RUN_NAME}/" "${LOCAL_CKPT_DIR}/"

echo "[train_remote] 완료: ${LOCAL_CKPT_DIR}"
