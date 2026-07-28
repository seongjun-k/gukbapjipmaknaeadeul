"""FastAPI app factory — 시연용 웹 UI 백엔드 (§7).

orch 덕타이핑 계약(INTERFACES.md):
  orch.machine.add_listener(cb), orch.machine.state, orch.machine.step_times
  orch.request_start()/request_stop()/request_retry()/request_reset() -> (ok, detail)
  orch.get_map_png() -> (png_bytes, meta_dict) | (None, None)
  orch.latest_pose -> dict | None, orch.latest_plan -> list | None, orch.obstacle -> bool
  orch.get_goal() -> {x,y,yaw}, orch.request_set_goal(x,y,yaw) -> (ok, detail)
  orch.request_set_initialpose(x,y,yaw) -> (ok, detail)
"""
from __future__ import annotations

import asyncio
import base64
import os
import time

import cv2
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse

_STREAM_FPS = 15
_PUSH_HZ = 5

_INDEX_CANDIDATES = [
    # ament share 경로는 _find_index_html이 별도 확인, 여기는 소스 트리 상대 경로 폴백
    os.path.join(os.path.dirname(__file__), "..", "web", "static", "index.html"),
]


def _find_index_html() -> str | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        p = os.path.join(get_package_share_directory("shelfbot"), "web", "static", "index.html")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    for c in _INDEX_CANDIDATES:
        if os.path.isfile(c):
            return c
    return None


# 비동기 제너레이터여야 함: 동기 제너레이터는 클라이언트 접속 종료 시 취소되지 않아
# 접속마다 스레드가 영원히 남는다 (starlette는 async gen만 disconnect 시 정리)
async def _mjpeg_generator(hub, name: str):
    period = 1.0 / _STREAM_FPS
    while True:
        frame = hub.get(name)
        if frame is None:
            await asyncio.sleep(0.1)
            continue
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            await asyncio.sleep(period)
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
        )
        await asyncio.sleep(period)


def create_app(orch, hub) -> FastAPI:
    app = FastAPI()

    @app.post("/start")
    def start():
        ok, detail = orch.request_start()
        status = 200 if ok else 409
        return JSONResponse({"ok": ok, "detail": detail}, status_code=status)

    @app.post("/stop")
    def stop():
        ok, detail = orch.request_stop()
        return JSONResponse({"ok": ok, "detail": detail}, status_code=200 if ok else 409)

    @app.post("/retry")
    def retry():
        ok, detail = orch.request_retry()
        return JSONResponse({"ok": ok, "detail": detail}, status_code=200 if ok else 409)

    @app.post("/reset")
    def reset():
        ok, detail = orch.request_reset()
        return JSONResponse({"ok": ok, "detail": detail}, status_code=200 if ok else 409)

    @app.get("/stream/{name}")
    def stream(name: str):
        return StreamingResponse(
            _mjpeg_generator(hub, name),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/map")
    def get_map():
        png, meta = orch.get_map_png()
        if png is None:
            return JSONResponse({"png_base64": None, "meta": None}, status_code=200)
        return JSONResponse({"png_base64": base64.b64encode(png).decode("ascii"), "meta": meta})

    @app.get("/goal")
    def get_goal():
        return JSONResponse(orch.get_goal())

    @app.post("/goal")
    async def set_goal(request: Request):
        body = await request.json()
        try:
            x, y, yaw = float(body["x"]), float(body["y"]), float(body["yaw"])
        except (KeyError, TypeError, ValueError):
            return JSONResponse({"ok": False, "detail": "invalid_body"}, status_code=400)
        ok, detail = orch.request_set_goal(x, y, yaw)
        return JSONResponse({"ok": ok, "detail": detail}, status_code=200 if ok else 409)

    @app.post("/initialpose")
    async def set_initialpose(request: Request):
        body = await request.json()
        try:
            x, y, yaw = float(body["x"]), float(body["y"]), float(body["yaw"])
        except (KeyError, TypeError, ValueError):
            return JSONResponse({"ok": False, "detail": "invalid_body"}, status_code=400)
        ok, detail = orch.request_set_initialpose(x, y, yaw)
        return JSONResponse({"ok": ok, "detail": detail}, status_code=200 if ok else 409)

    @app.get("/", response_class=HTMLResponse)
    def index():
        path = _find_index_html()
        if path is None:
            return HTMLResponse("<h1>index.html not found</h1>", status_code=500)
        with open(path, "r", encoding="utf-8") as f:
            return HTMLResponse(f.read())

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        loop = asyncio.get_event_loop()
        queue: asyncio.Queue = asyncio.Queue()

        def on_transition(event: dict):
            loop.call_soon_threadsafe(queue.put_nowait, {"type": "state", **event})

        orch.machine.add_listener(on_transition)

        async def pose_plan_pusher():
            period = 1.0 / _PUSH_HZ
            while True:
                await asyncio.sleep(period)
                msg = {
                    "type": "pose_plan",
                    "robot_pose": getattr(orch, "latest_pose", None),
                    "plan": getattr(orch, "latest_plan", None),
                    "obstacle": getattr(orch, "obstacle", False),
                }
                await queue.put(msg)

        pusher_task = asyncio.ensure_future(pose_plan_pusher())
        try:
            while True:
                msg = await queue.get()
                await websocket.send_json(msg)
        except WebSocketDisconnect:
            pass
        finally:
            pusher_task.cancel()
            # 접속 종료 시 리스너 제거 — 안 하면 접속마다 누적 + 닫힌 루프로 콜백
            if on_transition in orch.machine.listeners:
                orch.machine.listeners.remove(on_transition)

    return app
