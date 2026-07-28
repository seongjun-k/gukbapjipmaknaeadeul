"""web_server.py 계약 테스트 — 가짜 orch/hub로 rclpy 없이 검증."""
import numpy as np
from fastapi.testclient import TestClient

from gukbapjipmaknaeadeul.web_server import create_app


class FakeMachine:
    def __init__(self):
        self.state = "READY"
        self.step_times = {}
        self._listeners = []

    def add_listener(self, cb):
        self._listeners.append(cb)


class FakeOrch:
    def __init__(self):
        self.machine = FakeMachine()
        self.latest_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.latest_plan = [[0.0, 0.0], [1.0, 1.0]]
        self.obstacle = False
        self._running = False
        self._goal = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def request_start(self):
        if self._running:
            return False, "already running"
        self._running = True
        return True, "started"

    def request_stop(self):
        self._running = False
        return True, "stopped"

    def request_retry(self):
        return True, "retrying"

    def request_reset(self):
        return True, "reset"

    def get_map_png(self):
        return b"\x89PNG\r\n", {"resolution": 0.05, "origin": [0, 0], "width": 10, "height": 10}

    def get_goal(self):
        return self._goal

    def request_set_goal(self, x, y, yaw):
        self._goal = {"x": x, "y": y, "yaw": yaw}
        return True, "goal_set"

    def request_set_initialpose(self, x, y, yaw):
        return True, "initialpose_set"


class FakeHub:
    def get(self, name):
        if name == "hand":
            return np.zeros((4, 4, 3), dtype=np.uint8)
        return None

    def put(self, name, frame):
        pass


def make_client():
    orch = FakeOrch()
    hub = FakeHub()
    app = create_app(orch, hub)
    return TestClient(app), orch


def test_start_then_duplicate_409():
    client, orch = make_client()
    r1 = client.post("/start")
    assert r1.status_code == 200
    assert r1.json()["ok"] is True

    r2 = client.post("/start")
    assert r2.status_code == 409
    assert r2.json()["ok"] is False


def test_stop_retry_reset():
    client, orch = make_client()
    assert client.post("/stop").status_code == 200
    assert client.post("/retry").status_code == 200
    assert client.post("/reset").status_code == 200


def test_map_json_shape():
    client, orch = make_client()
    r = client.get("/map")
    assert r.status_code == 200
    body = r.json()
    assert "png_base64" in body and "meta" in body
    assert body["meta"]["width"] == 10


def test_stream_multipart_chunk():
    # 무한 MJPEG 스트림은 TestClient로 소비하면 종료 시 행 걸림(starlette 0.27 한계)
    # → 제너레이터를 직접 1청크만 검증
    import asyncio
    from gukbapjipmaknaeadeul.web_server import _mjpeg_generator

    chunk = asyncio.run(_mjpeg_generator(FakeHub(), "hand").__anext__())
    assert chunk.startswith(b"--frame\r\n")
    assert b"Content-Type: image/jpeg" in chunk


def test_stream_route_registered():
    client, orch = make_client()
    paths = [r.path for r in client.app.routes]
    assert "/stream/{name}" in paths


def test_index_returns_html():
    client, orch = make_client()
    r = client.get("/")
    assert r.status_code == 200
    assert "text/html" in r.headers["content-type"]


def test_goal_get_and_set():
    client, orch = make_client()
    r = client.get("/goal")
    assert r.status_code == 200
    assert r.json() == {"x": 0.0, "y": 0.0, "yaw": 0.0}

    r2 = client.post("/goal", json={"x": 1.0, "y": 2.0, "yaw": 0.5})
    assert r2.status_code == 200
    assert client.get("/goal").json() == {"x": 1.0, "y": 2.0, "yaw": 0.5}


def test_goal_invalid_body_400():
    client, orch = make_client()
    r = client.post("/goal", json={"x": 1.0})
    assert r.status_code == 400


def test_initialpose_set():
    client, orch = make_client()
    r = client.post("/initialpose", json={"x": 0.0, "y": 0.0, "yaw": 0.0})
    assert r.status_code == 200
    assert r.json()["ok"] is True
