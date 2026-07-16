"""NavigateToPose 액션 클라이언트 래퍼."""
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavClient:
    def __init__(self, node: Node):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None

    def send_goal(self, x: float, y: float, yaw: float, result_cb, timeout_sec: float) -> None:
        """goal 전송. 완료(성공/실패/타임아웃) 시 result_cb(ok: bool, reason: str) 호출."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            result_cb(False, "nav_server_unavailable")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, yaw)

        self._timed_out = False
        self._timer = self._node.create_timer(timeout_sec, lambda: self._on_timeout(result_cb))

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, result_cb))

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _on_timeout(self, result_cb) -> None:
        self._timed_out = True
        self._stop_timer()
        self.cancel()
        result_cb(False, "nav_timeout")

    def _on_goal_response(self, future, result_cb) -> None:
        if self._timed_out:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._stop_timer()
            result_cb(False, "nav_rejected")
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_result(f, result_cb))

    def _on_result(self, future, result_cb) -> None:
        if self._timed_out:
            return
        self._stop_timer()
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            result_cb(True, "")
        else:
            result_cb(False, "nav_aborted")

    def _stop_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
