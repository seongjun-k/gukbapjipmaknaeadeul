"""MJPEG HTTP 스트림용 lerobot 카메라 타입('mjpeg') 등록 — frame hub 캠 경합 회피용.

OpenCVCamera는 연결 시 cv2 set(width/fps)이 성공해야 하는데 HTTP 스트림엔
set()이 항상 실패해 연결이 거부된다(실측: width_success=False). robot config는
width/height/fps 명시가 필수라 생략도 불가 → 설정 검증만 완화한 서브클래스.
run_policy_wrapper.sh 가 이 모듈을 임포트한 뒤 lerobot-rollout main을 호출한다.
"""
from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig


@CameraConfig.register_subclass("mjpeg")
@dataclass
class MjpegCameraConfig(OpenCVCameraConfig):
    # 부모의 int|Path 선언이면 draccus가 URL을 PosixPath로 강제해 'http://'가 'http:/'로 뭉개짐
    index_or_path: str = ""


class MjpegCamera(OpenCVCamera):
    def _configure_capture_settings(self) -> None:
        # ponytail: 스트림엔 cv2 set()이 안 먹힘 — 프레임 크기/fps는 서버(frame hub) 설정을 신뢰
        self.capture_width, self.capture_height = self.config.width, self.config.height
        self.width, self.height = self.config.width, self.config.height
        self.fps = self.config.fps
