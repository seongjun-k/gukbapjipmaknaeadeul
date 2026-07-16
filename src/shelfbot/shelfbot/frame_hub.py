"""캠당 리더 스레드 1개로 최신 프레임만 유지하는 공유 버퍼 (rclpy 비의존).

USB 캠은 다중 오픈이 안 되므로 도킹 루프·MJPEG 스트림·정책 추론이 이 버퍼를 공유해 읽는다.
"""
from __future__ import annotations

import logging
import threading
import time

import cv2

logger = logging.getLogger(__name__)


class FrameHub:
    def __init__(self, cameras: dict):
        # cameras: {name: {device, width, height, fps}}
        self._cameras = cameras
        self._frames: dict[str, "object"] = {}
        self._locks: dict[str, threading.Lock] = {name: threading.Lock() for name in cameras}
        self._threads: dict[str, threading.Thread] = {}
        self._stop = threading.Event()

    def start(self) -> None:
        for name, cfg in self._cameras.items():
            t = threading.Thread(target=self._reader_loop, args=(name, cfg), daemon=True)
            self._threads[name] = t
            t.start()

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads.values():
            t.join(timeout=2.0)

    def get(self, name: str):
        lock = self._locks.get(name)
        if lock is None:
            return None
        with lock:
            frame = self._frames.get(name)
            return None if frame is None else frame.copy()

    def put(self, name: str, frame) -> None:
        # orchestrator가 ROS 이미지 토픽(예: pinky 카메라)을 직접 주입할 때 사용
        lock = self._locks.setdefault(name, threading.Lock())
        with lock:
            self._frames[name] = frame

    def _reader_loop(self, name: str, cfg: dict) -> None:
        device = cfg.get("device")
        cap = cv2.VideoCapture(device)
        if cfg.get("width"):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["width"])
        if cfg.get("height"):
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["height"])
        if cfg.get("fps"):
            cap.set(cv2.CAP_PROP_FPS, cfg["fps"])

        if not cap.isOpened():
            logger.warning("frame_hub: 카메라 '%s' (%s) 열기 실패 — 해당 캠 비활성", name, device)
            return

        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                with self._locks[name]:
                    self._frames[name] = frame
        finally:
            cap.release()
