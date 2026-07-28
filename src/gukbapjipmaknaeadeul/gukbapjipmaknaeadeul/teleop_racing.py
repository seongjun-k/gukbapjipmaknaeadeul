"""엑스박스 패드(USB 유선, xpad) 레이싱 방식 텔레옵: RT 가속, LT 브레이크/후진, 왼스틱 좌우 조향, A/X 기어."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# xpad 축: 0=왼스틱X, 2=LT, 5=RT (트리거는 놓으면 +1.0, 끝까지 밟으면 -1.0) / 버튼: 0=A, 2=X
AX_LSX, AX_LT, AX_RT = 0, 2, 5
BTN_A, BTN_X = 0, 2
# 기어별 최고 직진 속도(m/s). 하드웨어 한계 0.28(펌웨어 MAX_RPM=100, 바퀴 r=0.027)
GEARS = [0.1, 0.18, 0.28]
MAX_ANG = 1.2   # rad/s


class TeleopRacing(Node):
    def __init__(self):
        super().__init__('teleop_racing')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self._cb, 10)
        # xpad 트리거는 첫 입력 전까지 0.0(=반쯤 밟음)으로 보고됨 — 해제값 확인 전엔 무시(오발진 방지)
        self._trig_ready = {AX_LT: False, AX_RT: False}
        self._gear = 0
        self._prev_btn = (0, 0)  # (A, X) 직전 상태 — 눌림 엣지 검출용

    def _trig(self, axes, i):
        if not self._trig_ready[i]:
            if axes[i] > 0.9:
                self._trig_ready[i] = True
            return 0.0
        return (1.0 - axes[i]) / 2.0  # 0(놓음)~1(끝까지)

    def _cb(self, msg):
        a, x = msg.buttons[BTN_A], msg.buttons[BTN_X]
        if a and not self._prev_btn[0]:
            self._gear = min(self._gear + 1, len(GEARS) - 1)
            self.get_logger().info(f'기어 {self._gear + 1}')
        if x and not self._prev_btn[1]:
            self._gear = max(self._gear - 1, 0)
            self.get_logger().info(f'기어 {self._gear + 1}')
        self._prev_btn = (a, x)

        max_lin = GEARS[self._gear]
        t = Twist()
        lin = (self._trig(msg.axes, AX_RT) - self._trig(msg.axes, AX_LT)) * max_lin
        t.linear.x = max(-max_lin, min(max_lin, lin))
        t.angular.z = max(-MAX_ANG, min(MAX_ANG, msg.axes[AX_LSX] * MAX_ANG))
        self.pub.publish(t)


def main():
    rclpy.init()
    node = TeleopRacing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # 종료 시(예외 포함) 정지 보장
        rclpy.try_shutdown()
