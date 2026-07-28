"""ShelfBot orchestrator: rclpy 노드 + FastAPI 웹서버를 한 프로세스에서 실행.

스레드 구성: rclpy spin(메인 스레드) + uvicorn(데몬 스레드).
"""
import math
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from shelfbot.aruco_docking import ArucoDocking
from shelfbot.frame_hub import FrameHub
from shelfbot.machine import State, StateMachine
from shelfbot.nav_client import NavClient
from shelfbot.web_server import create_app


def _load_yaml(package_share_relative: str, filename: str) -> dict:
    """ament_index share 디렉토리에서 로드, 실패 시 소스 트리 상대 경로 폴백."""
    try:
        share_dir = get_package_share_directory('shelfbot')
        path = os.path.join(share_dir, package_share_relative, filename)
        if os.path.exists(path):
            with open(path) as f:
                return yaml.safe_load(f)
    except Exception:
        pass
    # 소스 트리 폴백 (colcon build 전 개발 중)
    fallback = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        package_share_relative, filename,
    )
    with open(fallback) as f:
        return yaml.safe_load(f)


# 런타임 튜닝 허용 파라미터 (ArucoDocking cfg 키 부분집합) — booth.yaml docking 키 이름과 결합(SSoT)
_DOCKING_PARAM_KEYS = (
    'kp_lin', 'kp_ang', 'max_lin', 'max_ang', 'pos_tol_m',
    'yaw_tol_deg', 'aligned_frames', 'lost_frames', 'timeout_sec',
)

# 상태 -> 핑키 LCD 표시 텍스트 (NAVIGATING은 obstacle 여부로 별도 분기)
_STATUS_TEXT = {
    'READY': '대기',
    'DOCKING': '도착',
    'PLACING': '도착',
    'DONE': '완료',
    'FAILED': '오류',
}


class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')

        self.cfg = _load_yaml('config', 'booth.yaml')
        try:
            self.cameras_cfg = _load_yaml('config', 'cameras.yaml')
        except Exception:
            self.cameras_cfg = {}

        self.machine = StateMachine(log_dir=os.path.expanduser('~/shelfbot/logs'))
        self.machine.add_listener(self._on_transition)

        topics = self.cfg['topics']
        self._cmd_vel_pub = self.create_publisher(Twist, topics['cmd_vel'], 10)
        self.create_subscription(OccupancyGrid, topics['map'], self._on_map, 1)
        self.create_subscription(PoseWithCovarianceStamped, topics['pose'], self._on_pose, 10)
        self.create_subscription(Path, topics['plan'], self._on_plan, 10)
        self.create_subscription(LaserScan, topics['scan'], self._on_scan, 10)

        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, topics['initialpose'], 10)
        # 늦게 뜨는 LCD 노드도 마지막 상태를 받도록 TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_text_pub = self.create_publisher(String, topics['status_text'], status_qos)

        self.nav_client = NavClient(self)
        calib_file = os.path.expanduser(self.cfg['docking'].get('calib_file') or '')
        if calib_file and os.path.isfile(calib_file):
            calib = np.load(calib_file)
            camera_matrix, dist_coeffs = calib['K'], calib['dist']
        else:
            # 캘리브레이션 전에는 도킹 불가 (solvePnP가 K 필요) — 진입 시 즉시 실패시킴
            camera_matrix, dist_coeffs = None, None
            self.get_logger().warning(f'docking.calib_file 없음({calib_file!r}) — 도킹 비활성')
        self.docking = ArucoDocking(
            self.cfg['docking'],
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )

        self.hub = FrameHub(self.cameras_cfg.get('cameras', {}))
        self.hub.start()

        self._map_png: bytes | None = None
        self._map_meta: dict | None = None
        self.latest_pose = None
        self.latest_plan = None
        self.obstacle = False
        self._goal_override: dict | None = None

        self._dock_timer = None
        self._placing_proc: subprocess.Popen | None = None
        self._placing_timer = None
        self._lock = threading.Lock()

        self._publish_status_text()  # 시작 시 현재(READY) 상태를 즉시 송출

    # ---------------- 상태 전이 리스너 ----------------

    def _on_transition(self, payload: dict) -> None:
        state = payload['state']
        self._publish_status_text()
        if state == State.NAVIGATING.value:
            self._start_navigating()
        elif state == State.DOCKING.value:
            self._start_docking()
        elif state == State.PLACING.value:
            self._start_placing()

    # ---------------- NAVIGATING ----------------

    def _start_navigating(self) -> None:
        goal = self._goal_override or self.cfg['goal_pose']
        timeout = self.cfg['nav']['timeout_sec']
        self.nav_client.send_goal(goal['x'], goal['y'], goal['yaw'], self._on_nav_result, timeout)

    def _on_nav_result(self, ok: bool, reason: str) -> None:
        if ok:
            self.machine.transition(State.DOCKING)
        else:
            self.machine.fail(State.NAVIGATING, reason or 'nav_failed')

    # ---------------- DOCKING ----------------

    def _start_docking(self) -> None:
        self.docking.reset()
        cfg = self.cfg['docking']
        rate_hz = cfg['rate_hz']
        self._dock_deadline = time.monotonic() + cfg['timeout_sec']
        self._dock_timer = self.create_timer(1.0 / rate_hz, self._docking_step)

    def _docking_step(self) -> None:
        try:
            frame = self.hub.get('top')
            if frame is None:
                return
            if time.monotonic() > self._dock_deadline:
                self._stop_docking()
                self.machine.fail(State.DOCKING, 'dock_timeout')
                return

            vx, wz, status = self.docking.step(frame)
            self._publish_cmd_vel(vx, wz)

            if status == 'aligned':
                self._stop_docking()
                self.machine.transition(State.PLACING)
            elif status == 'lost':
                self._stop_docking()
                self.machine.fail(State.DOCKING, 'marker_lost')
        except Exception:
            # 예외 발생 시에도 반드시 정지 퍼블리시 (§4.2 안전 규칙)
            self._stop_docking()
            raise

    def _stop_docking(self) -> None:
        if self._dock_timer is not None:
            self._dock_timer.cancel()
            self._dock_timer = None
        self._publish_cmd_vel(0.0, 0.0)

    def _publish_cmd_vel(self, vx: float, wz: float) -> None:
        max_lin = self.cfg['docking']['max_lin']
        max_ang = self.cfg['docking']['max_ang']
        vx = max(-max_lin, min(max_lin, vx))
        wz = max(-max_ang, min(max_ang, wz))
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self._cmd_vel_pub.publish(msg)

    # ---------------- PLACING ----------------

    def _start_placing(self) -> None:
        cmd = self.cfg['placing']['cmd']
        timeout = self.cfg['placing']['timeout_sec']
        self._placing_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        self._placing_deadline = time.monotonic() + timeout
        self._placing_timer = self.create_timer(0.5, self._placing_poll)

    def _placing_poll(self) -> None:
        proc = self._placing_proc
        if proc is None:
            return
        ret = proc.poll()
        if ret is None:
            if time.monotonic() > self._placing_deadline:
                proc.kill()
                self._stop_placing_timer()
                self.machine.fail(State.PLACING, 'place_timeout')
            return
        self._stop_placing_timer()
        if ret == 0:
            self.machine.transition(State.DONE)
        else:
            self.machine.fail(State.PLACING, 'place_failed')

    def _stop_placing_timer(self) -> None:
        if self._placing_timer is not None:
            self._placing_timer.cancel()
            self._placing_timer = None

    # ---------------- 구독 콜백 (웹용) ----------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 128
        img[data == 0] = 255
        img[data > 0] = 0
        img = np.flipud(img)
        ok, buf = cv2.imencode('.png', img)
        if ok:
            self._map_png = buf.tobytes()
            self._map_meta = {
                'resolution': msg.info.resolution,
                'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y},
                'width': w,
                'height': h,
            }

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.latest_pose = {'x': p.x, 'y': p.y, 'qz': o.z, 'qw': o.w}

    def _on_plan(self, msg: Path) -> None:
        self.latest_plan = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]

    def _on_scan(self, msg: LaserScan) -> None:
        # 전방 ±fov_deg/2 내 유효 최소거리 < dist_m 이면 장애물로 판단 (NAVIGATING에서만 의미)
        cfg = self.cfg['obstacle']
        half_fov = math.radians(cfg['fov_deg']) / 2.0
        min_dist = None
        angle = msg.angle_min
        for r in msg.ranges:
            if -half_fov <= angle <= half_fov and msg.range_min <= r <= msg.range_max:
                if min_dist is None or r < min_dist:
                    min_dist = r
            angle += msg.angle_increment
        new_obstacle = min_dist is not None and min_dist < cfg['dist_m']
        if new_obstacle != self.obstacle:
            self.obstacle = new_obstacle
            if self.machine.state == State.NAVIGATING:
                self._publish_status_text()

    def _publish_status_text(self) -> None:
        state = self.machine.state.value
        if state == State.NAVIGATING.value:
            text = '물체감지 대기중' if self.obstacle else '가는중'
        else:
            text = _STATUS_TEXT.get(state, state)
        msg = String()
        msg.data = text
        self._status_text_pub.publish(msg)

    def get_map_png(self):
        if self._map_png is None:
            return None, None
        return self._map_png, self._map_meta

    # ---------------- 웹 요청 핸들러 ----------------

    def request_start(self):
        with self._lock:
            ok = self.machine.start()
        return ok, ('started' if ok else 'not_ready')

    def request_stop(self):
        with self._lock:
            self._stop_docking()
            self.nav_client.cancel()
            if self._placing_proc is not None and self._placing_proc.poll() is None:
                self._placing_proc.terminate()
            self._stop_placing_timer()
            if self.machine.state in (State.NAVIGATING, State.DOCKING, State.PLACING):
                self.machine.fail(self.machine.state, 'stopped')
        return True, 'stopped'

    def request_retry(self):
        with self._lock:
            target = self.machine.retry()
        return (target is not None), (f'retrying_{target.value}' if target else 'not_failed')

    def request_reset(self):
        with self._lock:
            ok = self.machine.reset()
        return ok, ('reset' if ok else 'not_resettable')

    def get_goal(self) -> dict:
        return self._goal_override or self.cfg['goal_pose']

    def request_set_goal(self, x: float, y: float, yaw: float):
        with self._lock:
            self._goal_override = {'x': x, 'y': y, 'yaw': yaw}
        return True, 'goal_set'

    def request_set_initialpose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25   # x (AMCL 관례)
        cov[7] = 0.25   # y
        cov[35] = 0.068  # yaw
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        return True, 'initialpose_set'

    def request_get_params(self) -> dict:
        cfg = self.cfg['docking']
        return {k: cfg[k] for k in _DOCKING_PARAM_KEYS if k in cfg}

    def request_set_params(self, params: dict):
        with self._lock:
            if self.machine.state == State.DOCKING:
                return False, 'docking_in_progress'
            invalid_keys = [k for k in params if k not in _DOCKING_PARAM_KEYS]
            if invalid_keys:
                return False, f'invalid_keys:{",".join(invalid_keys)}'
            invalid_values = [
                k for k, v in params.items()
                if isinstance(v, bool) or not isinstance(v, (int, float))
            ]
            if invalid_values:
                return False, f'invalid_values:{",".join(invalid_values)}'
            self.cfg['docking'].update(params)
            # ArucoDocking은 __init__에서만 cfg를 읽으므로 동일 캘리브레이션으로 재생성
            self.docking = ArucoDocking(
                self.cfg['docking'],
                camera_matrix=self.docking.camera_matrix,
                dist_coeffs=self.docking.dist_coeffs,
            )
        return True, 'params_updated'

    def shutdown(self) -> None:
        self._stop_docking()
        if self._placing_proc is not None and self._placing_proc.poll() is None:
            self._placing_proc.terminate()
        self.hub.stop()


def main(args=None):
    rclpy.init(args=args)
    orch = Orchestrator()

    app = create_app(orch, orch.hub)
    web_cfg = orch.cfg['web']
    config = uvicorn.Config(app, host=web_cfg['host'], port=web_cfg['port'], log_level='warning')
    server = uvicorn.Server(config)
    web_thread = threading.Thread(target=server.run, daemon=True)
    web_thread.start()

    try:
        rclpy.spin(orch)
    except KeyboardInterrupt:
        pass
    finally:
        orch.shutdown()
        orch.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
