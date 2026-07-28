"""순수 파이썬 상태머신 (rclpy 의존 없음). orchestrator_node에서 사용.

상태 흐름: READY -> NAVIGATING -> PLACING -> DONE -> (reset) -> READY
           실패 시 어느 단계에서든 -> FAILED(reason) -> retry(실패 단계로) 또는 reset(READY)
"""
import json
import os
import time
from datetime import datetime
from enum import Enum


class State(str, Enum):
    READY = "READY"
    NAVIGATING = "NAVIGATING"
    PLACING = "PLACING"
    DONE = "DONE"
    FAILED = "FAILED"


# READY 이후 정상 진행 순서 (retry 시 실패 단계로 되돌아가기 위한 순서 정보로도 사용)
_SEQUENCE = [State.NAVIGATING, State.PLACING, State.DONE]


class StateMachine:
    def __init__(self, log_dir: str = "logs"):
        self.state = State.READY
        self.fail_reason: str | None = None
        self.failed_state: State | None = None
        self.step_times: dict[str, float] = {}
        self.listeners: list = []

        self._log_dir = log_dir
        self._log_path: str | None = None
        self._step_start: float | None = None

    def add_listener(self, cb) -> None:
        self.listeners.append(cb)

    def start(self) -> bool:
        """READY -> NAVIGATING만 허용."""
        if self.state != State.READY:
            return False
        os.makedirs(self._log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(self._log_dir, f"cycle_{ts}.jsonl")
        self.fail_reason = None
        self.failed_state = None
        self.step_times = {}
        self.transition(State.NAVIGATING)
        return True

    def transition(self, new: State, reason: str | None = None) -> None:
        now = time.monotonic()
        # 이전 단계 소요 시간 기록 (READY/FAILED 등 비계측 상태 전이는 제외)
        if self._step_start is not None and self.state != State.READY:
            self.step_times[self.state.value] = round(now - self._step_start, 3)
        self._step_start = now

        self.state = new
        if new == State.FAILED:
            self.fail_reason = reason
        self._log(reason)
        self._broadcast()

    def retry(self) -> State | None:
        """FAILED -> 실패했던 단계로 복귀. FAILED가 아니면 None."""
        if self.state != State.FAILED or self.failed_state is None:
            return None
        target = self.failed_state
        self.fail_reason = None
        self.failed_state = None
        self.transition(target)
        return target

    def reset(self) -> bool:
        """DONE/FAILED -> READY만 허용."""
        if self.state not in (State.DONE, State.FAILED):
            return False
        self.fail_reason = None
        self.failed_state = None
        self.transition(State.READY)
        return True

    def fail(self, failed_state: State, reason: str) -> None:
        """도우미: 실패 단계를 기록하며 FAILED로 전이."""
        self.failed_state = failed_state
        self.transition(State.FAILED, reason)

    def _log(self, reason: str | None) -> None:
        if not self._log_path:
            return
        entry = {
            "ts": datetime.now().isoformat(),
            "state": self.state.value,
            "reason": reason,
            "step_times": dict(self.step_times),
        }
        with open(self._log_path, "a") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

    def _broadcast(self) -> None:
        payload = {
            "state": self.state.value,
            "reason": self.fail_reason,
            "failed_state": self.failed_state.value if self.failed_state else None,
            "step_times": dict(self.step_times),
        }
        for cb in list(self.listeners):
            try:
                cb(payload)
            except Exception:
                # 리스너(예: 끊긴 웹소켓) 하나가 죽어도 상태 전이는 계속돼야 함
                pass
