"""PC workbench service: separate loopback control and observation connections."""
import asyncio
from contextlib import asynccontextmanager
import json
from pathlib import Path
import secrets
import sys
import time
from urllib.parse import urlparse
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import websockets
from .recording import EpisodeRecorder, read_raw_episode
from .model import ROOT


class GatewayConnection:
    def __init__(self, ws, hello):
        self.ws, self.status = ws, hello["status"]
        self.lock = asyncio.Lock()

    async def rpc(self, data):
        async with self.lock:
            payload = dict(data, session=self.status["session"])
            payload["pc_received_monotonic_ns"] = time.perf_counter_ns()
            if data.get("operation") == "input": payload["pc_input_monotonic_ns"] = time.perf_counter_ns()
            await self.ws.send(json.dumps(payload, allow_nan=False))
            reply = json.loads(await asyncio.wait_for(self.ws.recv(), 1))
            if 'rpc_timing' in reply:
                reply['rpc_timing'].update(pc_gateway_sent_ns=payload['pc_received_monotonic_ns'],pc_gateway_reply_ns=time.perf_counter_ns())
            if "status" in reply: self.status = reply["status"]
            return reply


def make_app(data_root=None, gateway="ws://127.0.0.1:8766", backend="simulation"):
    if backend == "hardware-observe":
        from .observation_service import make_app as observer
        return observer(data_root,gateway)
    if backend != "simulation": raise ValueError("Unknown backend")
    if urlparse(gateway).hostname not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError("This release only supports a loopback simulation gateway")
    root = Path(data_root or ROOT.parent / "datasets" / "e6_teleop").resolve()
    recorder = EpisodeRecorder(root / "raw")
    viewers = set()
    owner = None
    controller = None
    latest = None
    export_job = dict(state="idle")
    audit = dict(active=False, samples=[])


    async def stop(reason):
        if controller:
            try: await controller.rpc(dict(operation="stop"))
            except Exception: pass
        if recorder.active:
            recorder.incomplete = True
            recorder.finish("aborted", reason)

    @asynccontextmanager
    async def lifespan(app):
        async def observe():
            nonlocal latest
            while True:
                try:
                    async with websockets.connect(gateway + "/ws/observations", max_size=8*1024*1024, proxy=None) as ws:
                        async for raw in ws:
                            latest = json.loads(raw)
                            latest["pc_received_monotonic_ns"] = time.perf_counter_ns()
                            if "error" in latest:
                                await stop(latest["error"])
                            elif recorder.active:
                                try: recorder.sample(latest)
                                except Exception as exc: await stop(f"录制失败：{exc}")
                            public = dict(latest, recording=recorder.state())
                            if "images" in public:
                                public["images"] = dict(public["images"], wrist_png=public["images"].get("wrist_png"))
                            for queue in tuple(viewers):
                                if queue.full(): queue.get_nowait()
                                queue.put_nowait(public)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    await stop(f"模拟网关断开：{type(exc).__name__}")
                    latest = dict(error="模拟网关连接中断", recording=recorder.state())
                    for queue in tuple(viewers):
                        if queue.full(): queue.get_nowait()
                        queue.put_nowait(latest)
                    await asyncio.sleep(1)
        worker = asyncio.create_task(observe())
        yield
        await stop("工作台退出")
        worker.cancel()
        try: await worker
        except asyncio.CancelledError: pass

    app = FastAPI(lifespan=lifespan)

    @app.post('/api/input-audit/start')
    def audit_start():
        audit.update(active=True,samples=[])
        return dict(active=True,hardware_connected=False)

    @app.post('/api/input-audit/finish')
    def audit_finish():
        audit['active']=False
        path=root/'input_audit';path.mkdir(parents=True,exist_ok=True)
        name=str(time.perf_counter_ns())+'.json'
        (path/name).write_text(json.dumps(audit['samples'],ensure_ascii=False),encoding='utf-8')
        return dict(file=str(path/name),samples=len(audit['samples']),hardware_connected=False)

    @app.middleware("http")
    async def local_origin(request: Request, call_next):
        origin = request.headers.get("origin")
        if origin and urlparse(origin).hostname not in {"127.0.0.1", "localhost", "::1"}:
            return __import__("fastapi").responses.JSONResponse({"error": "Local origin required"}, status_code=403)
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Content-Security-Policy"] = "default-src 'self'; img-src 'self' data:; style-src 'self' 'unsafe-inline'; connect-src 'self' ws://127.0.0.1:8765 ws://localhost:8765; frame-ancestors 'none'"
        return response

    def episode_path(name):
        path = (root / "raw" / name).resolve()
        if not path.is_relative_to(root / "raw") or path.parent != root / "raw" or not path.is_dir():
            raise HTTPException(404, "Episode not found")
        return path

    @app.get("/api/state")
    def state():
        return dict(backend="simulation", hardware_connected=False, gateway_ready=bool(latest and "observation" in latest),
                    recording=recorder.state(), export=export_job)

    @app.get("/api/episodes")
    def episodes():
        result = []
        for path in sorted((root / "raw").glob("*"), reverse=True):
            if (path / "metadata.json").is_file() and path != (recorder.directory if recorder.active else None):
                metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
                summary, _ = read_raw_episode(path)
                result.append(dict(id=path.name, task=metadata["task"], **summary))
        return result

    @app.get("/api/episodes/{name}")
    def episode(name: str):
        path = episode_path(name)
        summary, rows = read_raw_episode(path)
        return dict(summary=summary, rows=[{k: row[k] for k in ("sample_index", "observation", "effective_action", "image")} for row in rows])

    @app.get("/api/episodes/{name}/images/{image}")
    def image(name: str, image: str):
        if len(image) != 10 or not image[:6].isdigit() or not image.endswith(".png"):
            raise HTTPException(404)
        return FileResponse(episode_path(name) / "images" / image)

    def dataset_path(name):
        path = (root / "lerobot" / name).resolve()
        if path.parent != root / "lerobot" or not (path / "simulation_validation.json").is_file():
            raise HTTPException(404, "Validated local dataset not found")
        return path

    @app.get("/api/datasets")
    def datasets():
        return [dict(id=path.name, **json.loads((path / "simulation_validation.json").read_text(encoding="utf-8")))
                for path in sorted((root / "lerobot").glob("*"), reverse=True) if (path / "simulation_validation.json").is_file()]

    @app.get("/api/datasets/{name}/frames/{index}")
    def dataset_frame(name: str, index: int):
        from .dataset_view import read_frame
        try: return read_frame(dataset_path(name), index)
        except ValueError as exc: raise HTTPException(400, str(exc)) from exc

    @app.post("/api/export/{name}")
    async def export(name: str):
        path = episode_path(name)
        summary, _ = read_raw_episode(path)
        if not summary["complete"] or summary["result"] == "aborted":
            raise HTTPException(400, "中止或不完整episode不能导出")
        if export_job["state"] == "running": raise HTTPException(409, "已有导出正在运行")
        output = root / "lerobot" / name
        if output.exists(): raise HTTPException(409, "该episode已有导出目录，请使用现有结果")
        export_job.update(state="running", episode=name, output=str(output), log="")
        async def run():
            try:
                proc = await asyncio.create_subprocess_exec(sys.executable, "-m", "e6_teleop.exporter", str(path), "--output", str(output), stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
                log, _ = await proc.communicate()
                export_job.update(state="done" if proc.returncode == 0 else "failed", log=log.decode("utf-8", errors="replace")[-16000:])
            except Exception as exc:
                export_job.update(state="failed", log=str(exc))
        asyncio.create_task(run())
        return export_job

    @app.websocket("/ws/state")
    async def observations(ws: WebSocket):
        await ws.accept()
        queue = asyncio.Queue(maxsize=1)
        viewers.add(queue)
        try:
            while True: await ws.send_json(await queue.get())
        except (WebSocketDisconnect, RuntimeError): pass
        finally: viewers.discard(queue)

    @app.websocket("/ws/control")
    async def control(ws: WebSocket):
        nonlocal owner, controller
        origin = ws.headers.get("origin")
        if origin and urlparse(origin).hostname not in {"127.0.0.1", "localhost", "::1"}:
            await ws.close(code=1008); return
        await ws.accept()
        if owner is not None:
            await ws.send_json(dict(ok=False, error="已有页面持有控制权")); await ws.close(code=1008); return
        token = secrets.token_urlsafe(12)
        owner = token
        try:
            async with websockets.connect(gateway + "/ws/control", proxy=None) as upstream:
                hello = json.loads(await upstream.recv())
                if not hello["ok"]:
                    await ws.send_json(hello); return
                controller = GatewayConnection(upstream, hello)
                await ws.send_json(hello)
                while True:
                    data = await ws.receive_json()
                    try:
                        if not isinstance(data, dict): raise ValueError("命令必须为对象")
                        operation = data.get("operation")
                        if operation == "record/start":
                            if not latest or "observation" not in latest: raise ValueError("模拟观测尚未就绪")
                            if (time.perf_counter_ns()-latest["pc_received_monotonic_ns"])/1e9 > .25: raise ValueError("模拟观测已过期")
                            manifest = json.loads((ROOT / "assets" / "e6" / "manifest.json").read_text(encoding="utf-8"))
                            recorder.start(data.get("task"), dict(model_manifest=manifest, limits=dict(translation_m_s=.01, rotation_rad_s=.0872664626, gripper_s=.1)))
                            reply = dict(ok=True, status=controller.status, recording=recorder.state())
                        elif operation == "record/finish":
                            await controller.rpc(dict(operation="stop"))
                            recorder.finish(data.get("result"))
                            reply = dict(ok=True, status=controller.status, recording=recorder.state())
                        else:
                            reply = await controller.rpc(data)
                        if audit['active'] and len(audit['samples'])<12000:
                            audit['samples'].append(dict(operation=operation,input={k:v for k,v in data.items() if k not in ('lease','session')},
                                                         status={k:v for k,v in reply.get('status',{}).items() if k not in ('lease','session')},
                                                         observation=latest.get('observation') if latest else None,
                                                         effective_action=latest.get('effective_action') if latest else None,
                                                         pc_monotonic_ns=time.perf_counter_ns()))
                        await ws.send_json(reply)
                    except Exception as exc:
                        if not isinstance(exc, (ValueError, KeyError, TypeError)):
                            await stop(f"工作台操作失败：{type(exc).__name__}: {exc}")
                        await ws.send_json(dict(ok=False, error=str(exc), status=controller.status))
        except (WebSocketDisconnect, RuntimeError): pass
        except Exception as exc:
            try: await ws.send_json(dict(ok=False, error=f"网关连接异常：{exc}"))
            except Exception: pass
        finally:
            if owner == token:
                await stop("控制页面断开")
                owner, controller = None, None

    dist = ROOT / "web" / "dist"
    if dist.exists(): app.mount("/", StaticFiles(directory=dist, html=True), name="web")
    return app
