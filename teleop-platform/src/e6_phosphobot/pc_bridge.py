"""Transparent PC proxy inside the full phosphobot application.

No PC motion integrator, additional lease, device socket, or action inference.
The X5 supplies capabilities/state and is the sole control owner.
"""
import asyncio
from dataclasses import asdict
from contextlib import asynccontextmanager
import json
import logging
import time
from pathlib import Path
from urllib.parse import urlparse
from uuid import uuid4

import numpy as np
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.responses import Response
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosedOK
from .intent import Limits
from .input_relay_timing import InputRelayTiming, relay_inputs


class StateCache:
    def __init__(self):
        self.value=None
        self.received_ns=None
    def update(self,value):
        if value.get('kind')!='state' or value.get('detail_level')=='control':return False
        self.value=value
        self.received_ns=time.perf_counter_ns()
        return True
    def current(self):
        if self.value is None or time.perf_counter_ns()-self.received_ns>500_000_000:
            raise HTTPException(503,'X5 observation unavailable')
        return self.value
    def capabilities(self):
        # Keep the configured E6 UI and local records available offline.
        # This never makes a stale observation or control session ready.
        if self.value is not None:return self.value['capabilities']
        return dict(enabled=True,backend='E6 未连接',original_recording=True,**asdict(Limits()))

    def diagnostics(self):
        age=None if self.received_ns is None else (time.perf_counter_ns()-self.received_ns)/1e6
        return dict(state=self.value, pc_received_ns=self.received_ns,
                    pc_age_ms=age, fresh=bool(age is not None and 0<=age<=500))


def make_display_robot(cache):
    from phosphobot.models.robot import BaseRobot
    class E6Display(BaseRobot):
        name='Dobot E6'
        SERIAL_ID='e6-x5-native'
        is_connected=True
        def read_joints_position(self,source='robot'):
            return np.array(cache.current()['actual']['joints_rad'])
        def get_observation(self,source='robot',do_forward=False):
            raise RuntimeError('Use E6 raw event recorder; do not fabricate actions or gripper state')
        def get_info_for_dataset(self):
            raise RuntimeError('Real LeRobot action export requires separate validation')
        async def connect(self):pass
        def disconnect(self):pass  # Does not touch physical enable or power.
        def blocked(self,*args,**kwargs):
            raise HTTPException(409,'E6 accepts only the prepared intent session')
        set_motors_positions=enable_torque=disable_torque=init_config=blocked
        async def move_robot_absolute(self,*args,**kwargs):self.blocked()
        async def move_to_initial_position(self):self.blocked()
        async def move_to_sleep(self):self.blocked()
    return E6Display()


def install_hardware(app,gateway_url,*,media_url='http://127.0.0.1:8777',episodes_dir=None,recordings_only=False):
    parsed=urlparse(gateway_url)
    if parsed.scheme!='ws' or parsed.hostname not in ('127.0.0.1','localhost','::1'):
        raise ValueError('Gateway must be accessed through a local SSH forward')
    from phosphobot.robot import get_rcm
    from phosphobot.endpoints.control import router as old_control
    from phosphobot.endpoints.recording import router as old_recording
    router=APIRouter()
    from .workspace_api import install_workspace_api
    install_workspace_api(router)
    cache=StateCache()
    input_timing=InputRelayTiming()
    from .episode_transfer import TransferWorker,rows,safe_name
    from .episodes import checked_id
    from .camera_archive import read_atomic_json
    media=urlparse(media_url)
    if media.scheme!='http' or media.hostname not in ('127.0.0.1','localhost','::1'):
        raise ValueError('Media must use its independent loopback SSH forward')
    episodes_dir=Path(episodes_dir or Path(__file__).resolve().parents[2]/'runtime/episodes')
    from .video_reports import install_video_reports
    install_video_reports(router, episodes_dir.parent/'diagnostics')
    from .problem_reports import install_problem_reports
    install_problem_reports(router, episodes_dir.parent.parent, cache.diagnostics)
    transfer=TransferWorker(media_url,episodes_dir)
    from .operator_logs import start_logging
    from .log_sources import LogSources
    from .log_api import install_log_api
    local_logs=LogSources()
    log_path=episodes_dir.parent/'logs'/('pc-'+local_logs.session+'.log')
    logger=logging.getLogger('e6.pc')
    local_logs.register('pc',log_path,root=episodes_dir.parent/'logs')
    install_log_api(router,media_url,local_logs)
    # Keep upstream status/selector integration. Never connect an upstream
    # serial driver, init/sleep routine, or alternate recorder to real hardware.
    get_rcm()._all_robots=[make_display_robot(cache)]
    allowed={'/joints/read','/robot/config','/robot/teleop-settings/read'}
    blocked={r.path for r in old_control.routes+old_recording.routes if r.path not in allowed}

    @app.middleware('http')
    async def block_incompatible_write_paths(request,call_next):
        if request.url.path in blocked:
            return JSONResponse({'detail':'该上游写入路径未适配E6；请使用正式意图控制页面'},status_code=409)
        return await call_next(request)

    async def observe():
        connected=False;reported_error=None
        while True:
            try:
                async with connect(gateway_url,compression=None,max_queue=1,open_timeout=3) as remote:
                    await remote.send(json.dumps({'role':'observe'}))
                    connected=True;reported_error=None;logger.info('X5 observation connected through control tunnel')
                    async for raw in remote:
                        cache.update(json.loads(raw))
            except asyncio.CancelledError:raise
            except Exception as exc:
                if connected or reported_error!=str(exc):logger.exception('X5 observation disconnected; reconnecting')
                connected=False;reported_error=str(exc)
                await asyncio.sleep(.5)

    original_lifespan=app.router.lifespan_context
    @asynccontextmanager
    async def lifespan(application):
        async with original_lifespan(application):
            _,log_sink=start_logging(log_path,'e6.pc')
            logger.info('PC application started; native log forwarding uses the independent media tunnel')
            observer=None if recordings_only else asyncio.create_task(observe())
            transfer.start()
            try:yield
            finally:
                if observer:
                    observer.cancel()
                    await asyncio.gather(observer,return_exceptions=True)
                await asyncio.to_thread(transfer.close)
                logger.info('PC application stopped')
                logger.removeHandler(log_sink);log_sink.close()
    app.router.lifespan_context=lifespan

    @router.get('/e6/capabilities')
    def capabilities():
        value=cache.capabilities()
        if recordings_only:value.update(enabled=False,backend='历史记录恢复（不连接设备）',recordings_only=True)
        return value

    @router.get('/e6/state')
    def state():
        value=cache.current()
        return dict(value,pc_received_ns=cache.received_ns)

    @router.get('/e6/diagnostics/input-relay')
    def input_relay_diagnostics():
        return input_timing.snapshot()

    @router.get('/e6/episodes')
    def episodes():
        snapshot=transfer.snapshot()
        items={item['episode']:item for item in snapshot['episodes']}
        for file in sorted(episodes_dir.glob('*/pc_transfer.json'),reverse=True):
            # A live failure remains visible even if its first status file
            # could not be created or its final atomic write failed.
            if file.parent.name not in items:
                items[file.parent.name]=read_atomic_json(file)
        for episode,item in items.items():
            for name in ('metadata','summary'):
                path=episodes_dir/episode/(name+'.json')
                if path.exists():item[name]=json.loads(path.read_text(encoding='utf-8'))
        return dict(episodes=[items[key] for key in sorted(items,reverse=True)],
                    transferring=snapshot['transferring'],error=snapshot['error'] or
                    ('部分历史目录无法恢复: '+str(snapshot['catalogue_errors'][:3]) if snapshot.get('catalogue_errors') else None),
                    catalogue_errors=snapshot.get('catalogue_errors',[]),recordings_only=recordings_only)

    def directory_of(episode):
        try:return episodes_dir/checked_id(episode)
        except ValueError as exc:raise HTTPException(400,str(exc))

    @router.post('/e6/episodes/{episode}/retry')
    def retry(episode:str):
        directory_of(episode);transfer.retry(episode);return dict(queued=True)

    @router.get('/e6/episodes/{episode}/samples')
    def episode_samples(episode:str,offset:int=0,limit:int=50):
        from .episode_browser import sample_page
        directory=directory_of(episode)
        if offset<0 or not 1<=limit<=100:raise HTTPException(400,'Invalid page')
        return sample_page(directory,offset,limit)

    @router.get('/e6/episodes/{episode}/rgb/{frame_id}')
    def episode_rgb(episode:str,frame_id:int):
        import zstandard
        directory=directory_of(episode)
        entry=next((r for r in rows(directory/'media/manifest.jsonl') if r['source']=='rgb' and r['id']==frame_id),None)
        if not entry:raise HTTPException(404,'Original frame unavailable')
        data=zstandard.ZstdDecompressor().decompress((directory/'media'/str(safe_name(entry['file']))).read_bytes(),max_output_size=entry['bytes'])
        return Response(data,media_type='image/jpeg')

    @router.websocket('/e6/control')
    async def control(ws:WebSocket):
        if recordings_only:
            await ws.close(code=1008)
            return
        origin=ws.headers.get('origin')
        if origin is not None and origin not in ('http://'+ws.headers.get('host',''),
                                                'https://'+ws.headers.get('host','')):
            await ws.close(code=1008)
            return
        await ws.accept()
        logger.info('Browser control connection opened')
        try:
            async with connect(gateway_url,compression=None,max_queue=1,
                               write_limit=4096,open_timeout=3) as remote:
                await remote.send(json.dumps({'role':'control'}))
                async def upstream():
                    await relay_inputs(ws,remote,input_timing,uuid4().hex)
                async def downstream():
                    async for raw in remote:
                        value=json.loads(raw)
                        if cache.update(value):value['pc_received_ns']=cache.received_ns
                        await ws.send_json(value)
                tasks=[asyncio.create_task(upstream()),asyncio.create_task(downstream())]
                try:
                    done,_=await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
                    for task in done:task.result()
                finally:
                    for task in tasks:task.cancel()
                    await asyncio.gather(*tasks,return_exceptions=True)
        # A normal close may follow an explicit refusal (e.g. second owner).
        # Do not replace the already-forwarded reason with a transport error.
        except (WebSocketDisconnect,asyncio.CancelledError,ConnectionClosedOK):pass
        except Exception as exc:
            try:await ws.send_json(dict(kind='state',state='未连接',ready=False,echo_ns=0,
                                       reason='X5连接已断开: '+str(exc)))
            except Exception:pass
        finally:
            logger.info('Browser control connection closed')
            try:await ws.close()
            except RuntimeError:pass

    app.router.routes[0:0]=router.routes
    return cache
