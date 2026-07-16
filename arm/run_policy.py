#!/usr/bin/env python3
"""soarm101 + ACT 정책 실행 스크립트 (lerobot venv 전용, torch 필요).

booth.yaml의 placing.cmd 가 이 스크립트를 서브프로세스로 호출한다
(주의: orchestrator_node.py는 arm/run_policy_wrapper.sh 를 부르므로
그 래퍼가 이 스크립트로 lerobot venv python을 지정해 실행해야 함 — A 소유 영역).

종료코드: 0 = 정상 완료(홈 자세 복귀), 1 = 실패/중단.
"""
import argparse
import signal
import sys


def parse_args():
    p = argparse.ArgumentParser(description="ACT 정책으로 soarm101 진열 동작 실행")
    p.add_argument("--checkpoint", required=True, help="LeRobot ACT 체크포인트 경로/ID")
    p.add_argument("--product", required=True, help="진열 상품 라벨 (sneakers/kitkat/freetime)")
    p.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="최대 정책 스텝 수 (기본: policy fps * 60초)",
    )
    return p.parse_args()


class PolicyRunner:
    """정책 로드/스텝/홈 복귀를 담당. torch·lerobot은 지연 임포트."""

    def __init__(self, checkpoint: str, product: str):
        self.checkpoint = checkpoint
        self.product = product
        self.policy = None
        self.robot = None
        self.cameras = None
        self._stop_requested = False

    def setup(self):
        # TODO: lerobot 버전마다 API가 다름 — 실제 설치 버전 확인 후 아래 임포트/생성 확정
        # from lerobot.common.policies.act.modeling_act import ACTPolicy
        # from lerobot.common.robot_devices.robots.factory import make_robot
        # self.policy = ACTPolicy.from_pretrained(self.checkpoint)
        # self.robot = make_robot("soarm101")  # config는 arm/ 하위 설정 파일 참조 (TBD)
        # self.robot.connect()
        # self.cameras = self.robot.cameras  # {"top": ..., "hand": ...} 등, cameras.yaml과 이름 맞출 것
        raise NotImplementedError("lerobot 실제 설치 후 구현 (torch 미설치 환경에서는 임포트 지연)")

    def policy_fps(self) -> int:
        # TODO: 체크포인트 config에서 실제 fps 읽기 (lerobot policy.config.fps 등)
        return 30

    def read_observation(self):
        # TODO: 캠 2대(top/hand) 프레임 + soarm101 관절 상태를 lerobot observation dict로 조립
        raise NotImplementedError

    def step(self, observation):
        # TODO: action = self.policy.select_action(observation); self.robot.send_action(action)
        raise NotImplementedError

    def at_home_pose(self) -> bool:
        # TODO: 관절 상태가 홈 자세 임계값 이내인지 판정 (종료 조건)
        raise NotImplementedError

    def go_home(self):
        # TODO: 안전 속도로 홈 자세 복귀 (SIGTERM 핸들러 및 정상 종료 공통 사용)
        pass

    def disconnect(self):
        if self.robot is not None:
            # TODO: self.robot.disconnect()
            pass

    def request_stop(self):
        self._stop_requested = True

    def run(self, max_steps: int) -> bool:
        """정책 루프. True면 홈 자세 복귀로 정상 종료, False면 max_steps 소진."""
        for _ in range(max_steps):
            if self._stop_requested:
                return False
            obs = self.read_observation()
            self.step(obs)
            if self.at_home_pose():
                return True
        return False


def main() -> int:
    args = parse_args()
    max_steps = args.max_steps or PolicyRunner("", "").policy_fps() * 60

    runner = PolicyRunner(args.checkpoint, args.product)

    def handle_sigterm(signum, frame):
        # 비상 정지: 정책 루프 중단 후 홈 복귀 시도
        runner.request_stop()

    signal.signal(signal.SIGTERM, handle_sigterm)

    try:
        runner.setup()
        completed = runner.run(max_steps)
        runner.go_home()
        return 0 if completed else 1
    except NotImplementedError as e:
        print(f"[run_policy] 미구현: {e}", file=sys.stderr)
        return 1
    except Exception as e:  # noqa: BLE001 - 서브프로세스 최상위, 실패를 종료코드로 전달
        print(f"[run_policy] 실패: {e}", file=sys.stderr)
        try:
            runner.go_home()
        except Exception:
            pass
        return 1
    finally:
        runner.disconnect()


if __name__ == "__main__":
    sys.exit(main())
