#!/usr/bin/env python3
"""핑키 CSI 카메라 MJPEG 스트리머 — 핑키(라즈베리파이)에서 실행.

웹 UI '핑키' 패널이 http://192.168.4.1:8081/stream 을 직접 표시한다.
ROS 토픽 대신 HTTP 직결인 이유: CSI 카메라는 V4L2 캡처가 안 되고(raw 토픽은 WiFi 대역폭 초과),
핑키에 camera_ros 미설치. picamera2 하드웨어 MJPEG 인코딩이 가장 가볍다.
"""
import io
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Condition

from libcamera import Transform
from picamera2 import Picamera2
# MJPEGEncoder(libav)는 핑키의 PyAV 버전과 비호환(qmin 제거) — 소프트웨어 JpegEncoder 사용
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

PORT = 8081


class StreamOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with output.condition:
                    output.condition.wait()
                    frame = output.frame
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):  # 접속 로그 소음 억제
        pass


picam2 = Picamera2()
# 핑키 카메라는 뒤집혀 장착됨(pinkylib camera.py의 ROTATE_180과 동일 보정)
# 색이 여전히 뒤집혀 보이면 "RGB888" ↔ "BGR888" 만 바꿔볼 것 (picamera2 포맷명이 메모리 순서와 반대인 기기 있음)
picam2.configure(picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}, transform=Transform(hflip=1, vflip=1)))
output = StreamOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))
try:
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
finally:
    picam2.stop_recording()
