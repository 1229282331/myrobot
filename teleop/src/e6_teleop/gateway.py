"""Loopback-only simulated gateway. It has no E6, CAN, SSH or camera SDK imports."""
import asyncio
import base64
import os
from contextlib import asynccontextmanager
from io import BytesIO
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from PIL import Image
from .core import GatewayCore, InputError
from .model import E6Model


def encoded(array, format):
    stream = BytesIO()
    Image.fromarray(array).save(stream, format, **({"compress_level": 1} if format == "PNG" else {"quality": 80}))
    return base64.b64encode(stream.getvalue()).decode("ascii")


def make_app(render=True):
    subscribers = set()

    @asynccontextmanager
    async def lifespan(app):
        model = E6Model(render=render)
        core = GatewayCore(model)
        app.state.core, app.state.latest = core, None

        async def loop():
            next_tick = asyncio.get_running_loop().time()
            while True:
                next_tick += core.settings.period_s
                try:
                    frame, images = core.tick()
                    frame["images"] = ({"wrist_png": encoded(images["wrist"], "PNG"), "overview_jpeg": encoded(images["overview"], "JPEG")}
                                       if render else {})
                    app.state.latest = frame
                    for queue in tuple(subscribers):
                        if queue.full():
                            queue.get_nowait()
                        queue.put_nowait(frame)
                except Exception as exc:
                    core.stop(f"模型或采图异常：{type(exc).__name__}: {exc}")
                    app.state.latest = dict(error=core.reason, status=core.status())
                    for queue in tuple(subscribers):
                        if queue.full(): queue.get_nowait()
                        queue.put_nowait(app.state.latest)
                await asyncio.sleep(max(0, next_tick-asyncio.get_running_loop().time()))
                if next_tick < asyncio.get_running_loop().time()-.15:
                    next_tick = asyncio.get_running_loop().time()

        worker = asyncio.create_task(loop())
        yield
        core.stop("网关退出")
        worker.cancel()
        try: await worker
        except asyncio.CancelledError: pass
        model.close()

    app = FastAPI(lifespan=lifespan)

    @app.get("/health")
    def health():
        return dict(app.state.core.status(), process_id=os.getpid(), instance=os.environ.get("E6_SIM_INSTANCE"))

    @app.websocket("/ws/control")
    async def control(ws: WebSocket):
        await ws.accept()
        core = app.state.core
        owner = None
        try:
            try:
                hello = core.connect()
                owner = hello["session"]
                await ws.send_json(dict(ok=True, status=hello))
            except InputError as exc:
                await ws.send_json(dict(ok=False, error=str(exc)))
                await ws.close(code=1008)
                return
            while True:
                data = await ws.receive_json()
                try:
                    if not isinstance(data, dict): raise InputError("命令必须为对象")
                    if data.get("session") != owner: raise InputError("会话与控制连接不匹配")
                    status = core.command(data)
                    await ws.send_json(dict(ok=True, status=status))
                except (ValueError, TypeError, KeyError) as exc:
                    await ws.send_json(dict(ok=False, error=str(exc), status=core.status()))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            core.disconnect(owner)

    @app.websocket("/ws/observations")
    async def observations(ws: WebSocket):
        await ws.accept()
        queue = asyncio.Queue(maxsize=1)
        subscribers.add(queue)
        try:
            while True:
                await ws.send_json(await queue.get())
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            subscribers.discard(queue)

    return app


app = make_app()
