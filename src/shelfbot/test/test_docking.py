"""ArucoDocking 순수 로직 테스트 (합성 프레임, ROS 미의존)."""
import cv2
import numpy as np
import pytest

from shelfbot.aruco_docking import ArucoDocking

MARKER_ID = 0
FRAME_SIZE = (480, 640)  # h, w
CAMERA_MATRIX = np.array(
    [[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]]
)
DIST_COEFFS = np.zeros(5)


def make_marker_frame(top_left=(220, 140), side=200):
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.drawMarker(dictionary, MARKER_ID, side)
    frame = np.full(FRAME_SIZE, 255, dtype=np.uint8)
    x0, y0 = top_left
    frame[y0 : y0 + side, x0 : x0 + side] = marker_img
    return frame


def make_blank_frame():
    return np.full(FRAME_SIZE, 255, dtype=np.uint8)


def base_cfg(**overrides):
    cfg = dict(
        marker_id=MARKER_ID,
        aruco_dict="DICT_4X4_50",
        marker_length_m=0.06,
        target_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        kp_lin=0.5,
        kp_ang=1.0,
        max_lin=0.05,
        max_ang=0.3,
        pos_tol_m=0.015,
        yaw_tol_deg=3.0,
        aligned_frames=5,
        lost_frames=4,
    )
    cfg.update(overrides)
    return cfg


def test_detect_and_pose_step_runs():
    docking = ArucoDocking(base_cfg(), CAMERA_MATRIX, DIST_COEFFS)
    frame = make_marker_frame()
    vx, wz, status = docking.step(frame)
    assert status in ("aligning", "aligned")
    assert isinstance(vx, float) and isinstance(wz, float)


def test_clamp_on_large_error():
    # 목표를 아주 멀리 두어 큰 오차를 유발, kp도 크게 잡아 클램프가 실제로 걸리는지 확인
    cfg = base_cfg(
        target_pose={"x": 5.0, "y": 5.0, "yaw": 90.0}, kp_lin=50.0, kp_ang=50.0,
        max_lin=0.05, max_ang=0.3,
    )
    docking = ArucoDocking(cfg, CAMERA_MATRIX, DIST_COEFFS)
    frame = make_marker_frame()
    vx, wz, status = docking.step(frame)
    assert status == "aligning"
    assert abs(vx) <= cfg["max_lin"] + 1e-9
    assert abs(wz) <= cfg["max_ang"] + 1e-9


def test_aligned_after_consecutive_frames():
    # 넉넉한 허용오차로, 검출되는 마커 pose가 곧 목표라고 간주되게 함
    cfg = base_cfg(
        target_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        pos_tol_m=10.0, yaw_tol_deg=180.0, aligned_frames=5,
    )
    docking = ArucoDocking(cfg, CAMERA_MATRIX, DIST_COEFFS)
    frame = make_marker_frame()

    for _ in range(cfg["aligned_frames"] - 1):
        _, _, status = docking.step(frame)
        assert status == "aligning"

    vx, wz, status = docking.step(frame)
    assert status == "aligned"
    assert vx == 0.0 and wz == 0.0


def test_lost_after_consecutive_missing_frames():
    cfg = base_cfg(lost_frames=4)
    docking = ArucoDocking(cfg, CAMERA_MATRIX, DIST_COEFFS)
    blank = make_blank_frame()

    for _ in range(cfg["lost_frames"] - 1):
        _, _, status = docking.step(blank)
        assert status == "searching"

    vx, wz, status = docking.step(blank)
    assert status == "lost"
    assert vx == 0.0 and wz == 0.0


def test_reset_clears_counters():
    cfg = base_cfg(
        target_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        pos_tol_m=10.0, yaw_tol_deg=180.0, aligned_frames=3,
    )
    docking = ArucoDocking(cfg, CAMERA_MATRIX, DIST_COEFFS)
    frame = make_marker_frame()

    docking.step(frame)
    docking.step(frame)
    docking.reset()

    for _ in range(cfg["aligned_frames"] - 1):
        _, _, status = docking.step(frame)
        assert status == "aligning"
    _, _, status = docking.step(frame)
    assert status == "aligned"
