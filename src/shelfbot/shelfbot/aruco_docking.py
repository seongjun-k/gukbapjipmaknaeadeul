"""아루코 마커 기반 도킹 제어 (순수 로직, rclpy 미의존).

cv2 4.6 기준 구 API 사용: Dictionary_get / DetectorParameters_create / detectMarkers.
(cv2.aruco.ArucoDetector 클래스는 4.7+ 전용이라 이 버전엔 없음)
"""
import math

import cv2
import numpy as np


class ArucoDocking:
    def __init__(self, cfg: dict, camera_matrix, dist_coeffs):
        self.cfg = cfg
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)

        dict_id = getattr(cv2.aruco, cfg["aruco_dict"])
        self.dictionary = cv2.aruco.Dictionary_get(dict_id)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.marker_id = cfg["marker_id"]
        self.marker_length = cfg["marker_length_m"]
        tp = cfg["target_pose"]
        self.target_x, self.target_y, self.target_yaw_deg = tp["x"], tp["y"], tp["yaw"]

        self.kp_lin = cfg["kp_lin"]
        self.kp_ang = cfg["kp_ang"]
        self.max_lin = cfg["max_lin"]
        self.max_ang = cfg["max_ang"]
        self.pos_tol_m = cfg["pos_tol_m"]
        self.yaw_tol_deg = cfg["yaw_tol_deg"]
        self.aligned_frames = cfg["aligned_frames"]
        self.lost_frames = cfg["lost_frames"]

        half = self.marker_length / 2.0
        # 마커 좌표계: 중심 원점, 시계반대 순서(코너 검출 순서와 일치: TL,TR,BR,BL)
        self._obj_points = np.array(
            [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
            dtype=np.float64,
        )

        self._aligned_count = 0
        self._lost_count = 0

    def reset(self) -> None:
        self._aligned_count = 0
        self._lost_count = 0

    def _clamp(self, vx: float, wz: float) -> tuple:
        vx = max(-self.max_lin, min(self.max_lin, vx))
        wz = max(-self.max_ang, min(self.max_ang, wz))
        return vx, wz

    def step(self, frame) -> tuple:
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.dictionary, parameters=self.detector_params
        )

        marker_corners = None
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                if int(i) == self.marker_id:
                    marker_corners = c
                    break

        if marker_corners is None:
            self._aligned_count = 0
            self._lost_count += 1
            if self._lost_count >= self.lost_frames:
                return 0.0, 0.0, "lost"
            return 0.0, 0.0, "searching"

        self._lost_count = 0

        ok, rvec, tvec = cv2.solvePnP(
            self._obj_points, marker_corners.reshape(4, 2), self.camera_matrix, self.dist_coeffs
        )
        if not ok:
            self._aligned_count = 0
            self._lost_count += 1
            if self._lost_count >= self.lost_frames:
                return 0.0, 0.0, "lost"
            return 0.0, 0.0, "searching"

        R, _ = cv2.Rodrigues(rvec)
        # 카메라 좌표계(x=우, y=하, z=전방) -> 로봇 평면(전방 x, 좌측 y)
        cur_x = float(tvec[2][0])
        cur_y = float(-tvec[0][0])
        cur_yaw = math.atan2(-R[0, 2], R[2, 2])

        e_x = self.target_x - cur_x
        e_y = self.target_y - cur_y
        e_pos = math.hypot(e_x, e_y)
        e_yaw_rad = math.radians(self.target_yaw_deg) - cur_yaw
        e_yaw_rad = math.atan2(math.sin(e_yaw_rad), math.cos(e_yaw_rad))
        e_yaw_deg = math.degrees(e_yaw_rad)

        if e_pos <= self.pos_tol_m and abs(e_yaw_deg) <= self.yaw_tol_deg:
            self._aligned_count += 1
        else:
            self._aligned_count = 0

        if self._aligned_count >= self.aligned_frames:
            return 0.0, 0.0, "aligned"

        # 차동구동 2단계 접근 (구현계획서 §5.2): 위치오차가 크면 목표방향으로
        # yaw를 틀며 전/후진해 횡오차를 수렴시키고, 근접하면 목표 yaw로 정렬.
        if e_pos > self.pos_tol_m * 2:
            angle_to_target = math.atan2(e_y, e_x)
            wz = self.kp_ang * angle_to_target
            vx = self.kp_lin * e_pos * math.cos(angle_to_target)
        else:
            wz = self.kp_ang * e_yaw_rad
            vx = self.kp_lin * e_pos

        vx, wz = self._clamp(vx, wz)
        return vx, wz, "aligning"
