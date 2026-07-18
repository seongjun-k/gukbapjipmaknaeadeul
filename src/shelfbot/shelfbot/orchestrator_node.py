"""ShelfBot orchestrator: rclpy 노드 + FastAPI 웹서버를 한 프로세스에서 실행.

스레드 구성: rclpy spin(메인 스레드) + uvicorn(데몬 스레드).
"""
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

        self._dock_timer = None
        self._placing_proc: subprocess.Popen | None = None
        self._placing_timer = None
        self._lock = threading.Lock()

    # ---------------- 상태 전이 리스너 ----------------

    def _on_transition(self, payload: dict) -> None:
        state = payload['state']
        if state == State.NAVIGATING.value:
            self._start_navigating()
        elif state == State.DOCKING.value:
            self._start_docking()
        elif state == State.PLACING.value:
            self._start_placing()

    # ---------------- NAVIGATING ----------------

    def _start_navigating(self) -> None:
        goal = self.cfg['goal_pose']
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
