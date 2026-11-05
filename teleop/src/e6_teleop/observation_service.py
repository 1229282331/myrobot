"""PC hardware bridge. Read-only v2 or startup-locked v3 control via the guardian."""
import asyncio
import base64
from contextlib import asynccontextmanager
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
import json
import secrets
from pathlib import Path
import time
import sys
import numpy as np
from urllib.parse import urlparse
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
import websockets
from .model import ROOT, E6Model
from .observation_recording import ObservationRecorder
from .observation_clock import TransportClock
from .observation_archive import begin_archive, retrieve_archive
sys.path.insert(0,str(ROOT/'device_gateway'))
from e6_observer.protocol import decode_wire,assemble


def make_app(data_root=None,gateway='ws://127.0.0.1:8767',render=True,archive_gateway=None,backend='hardware-observe',control_gateway=None,control_token=None,control_role='harness',media_gateway=None,pc_tcp_profile=None,exit_callback=None,exit_token=None):
    if backend not in ('hardware-observe','hardware-control'):raise ValueError('Unknown hardware backend')
    controlling=backend=='hardware-control'
    if controlling and (not control_gateway or not control_token):raise ValueError('Owned control gateway and token required')
    if control_gateway and urlparse(control_gateway).hostname not in {'localhost','127.0.0.1','::1'}:raise ValueError('Loopback control gateway required')
    if urlparse(gateway).hostname not in {'localhost','127.0.0.1','::1'}:raise ValueError('loopback gateway required')
    media_gateway=media_gateway or gateway
    if urlparse(media_gateway).hostname not in {'localhost','127.0.0.1','::1'}:raise ValueError('loopback media gateway required')
    archive_gateway=archive_gateway or gateway
    if urlparse(archive_gateway).hostname not in {'localhost','127.0.0.1','::1'}:raise ValueError('loopback archive gateway required')
    root=Path(data_root or ROOT.parent/'datasets'/'e6_teleop').resolve()
    dataset_kind='hardware_demonstration' if controlling else 'hardware_observation'
    raw_root=root/dataset_kind/'raw'
    recorder=ObservationRecorder(raw_root,dataset_kind)
    controller=None;owner=None;exiting=False;exit_confirmed=False
    async def stop_control():
        if controller and not exiting and not exit_confirmed:
            try:await controller.rpc(dict(operation='stop'))
            except Exception:pass
    latest=None;viewers=set();audit_viewers=set();incoming=asyncio.Queue(maxsize=4)
    gate=asyncio.Lock();preview_discarded=0;pc_error=None
    previous_instance=None
    last_received_ns=None
    first_valid=False
    media_latest={}
    finalizers=set()
    pipeline={'event_loop_lag_max_ms':0.0,'media_decode_max_ms':0.0,'queue_max_ms':0.0}
    def schedule_media(instance):
        if not recorder.pending:return
        async def finalize():
            nonlocal pc_error
            error=None
            try:await retrieve_archive(archive_gateway,instance,recorder)
            except asyncio.CancelledError:error='原始媒体回传被中断';raise
            except Exception as exc:error='原始媒体回传失败：'+str(exc);pc_error=error
            finally:
                try:await asyncio.to_thread(recorder.finalize_media,error)
                except Exception as exc:pc_error='原始媒体校验收尾失败：'+str(exc)
        task=asyncio.create_task(finalize());finalizers.add(task)
        task.add_done_callback(finalizers.discard)

    async def abort(reason):
        nonlocal pc_error
        pc_error=reason
        await stop_control()
        async with gate:
            if recorder.active:
                recorder.incomplete=True
                try:
                    try:await asyncio.to_thread(recorder.retain_failure,latest,reason)
                    except Exception:pass  # Still attempt the aborted summary if diagnostic IO failed.
                    await asyncio.to_thread(recorder.finish,'aborted',reason)
                    schedule_media(recorder.instance or previous_instance)
                except Exception as exc:pc_error=reason+'; 保存中止摘要失败：'+str(exc)
    def broadcast(frame):
        nonlocal preview_discarded
        public=dict(frame,recording=recorder.state(),pc_error=pc_error)
        if 'preview' in frame:public['preview']={n:{k:v for k,v in s.items() if k!='jpeg'} for n,s in frame['preview'].items()}
        public['sources']={n:{k:v for k,v in s.items() if k not in ('jpeg','uint16_raw','raw_base64','preview_jpeg')} for n,s in frame.get('sources',{}).items()}
        for queue in tuple(viewers):
            if queue.full():queue.get_nowait();preview_discarded+=1
            queue.put_nowait(public)

    @asynccontextmanager
    async def lifespan(app):
        nonlocal latest,pc_error,previous_instance
        model=E6Model(render=False)
        preview_model=E6Model(render=render)
        render_pool=ThreadPoolExecutor(max_workers=1)
        decode_pool=ThreadPoolExecutor(max_workers=2)
        kinematics_pool=ThreadPoolExecutor(max_workers=1)
        model.hide_attachments()
        preview_model.hide_attachments()
        cached_images={}
        renderer_report={}
        clock_ready=asyncio.Event()
        def render_snapshot(frame):
            images={}
            q=frame['observation'].get('joints_rad')
            if render and q:
                preview_model.set_state(q,0)
                im=preview_model.render('overview')
                if not renderer_report:
                    from OpenGL.GL import glGetString,GL_RENDERER,GL_VENDOR
                    renderer_report.update(renderer=glGetString(GL_RENDERER).decode(),vendor=glGetString(GL_VENDOR).decode())
                    if (pc_tcp_profile or {}).get('preview_renderer')=='mesa-software' and 'llvmpipe' not in renderer_report['renderer'].lower():
                        raise ValueError('PC model preview did not use the accepted software renderer')
                buf=BytesIO();Image.fromarray(im).save(buf,'JPEG',quality=80)
                images['overview_jpeg']=base64.b64encode(buf.getvalue()).decode()
            depth=frame['sources']['depth']
            if depth.get('uint16_raw'):
                raw=np.frombuffer(base64.b64decode(depth['uint16_raw'],validate=True),dtype='<u2').reshape(depth['height'],depth['width'])
                gray=np.clip(raw.astype(float)*depth['depth_scale_mm']/1500*255,0,255).astype('uint8')
                buf=BytesIO();Image.fromarray(gray).resize((320,240)).save(buf,'JPEG',quality=75)
                images['depth_jpeg']=base64.b64encode(buf.getvalue()).decode()
                images['depth_frame_id']=depth['device_frame_id']
            return images
        async def preview():
            nonlocal cached_images
            while True:
                await asyncio.sleep(.2)
                frame=latest
                if frame:
                    try:cached_images=await asyncio.get_running_loop().run_in_executor(render_pool,render_snapshot,frame)
                    except Exception as exc:
                        cached_images={'model_error':str(exc)}
                        if (pc_tcp_profile or {}).get('preview_renderer')=='mesa-software':await abort('已验收的软件模型渲染不可用：'+str(exc))
        async def receive_media():
            nonlocal media_latest
            await clock_ready.wait()
            while True:
                try:
                    async with websockets.connect(media_gateway+'/ws/media',max_size=8*1024*1024,proxy=None,compression=None,max_queue=2) as ws:
                        cache={}
                        async for message in ws:
                            decode_start=time.perf_counter_ns()
                            media=await asyncio.get_running_loop().run_in_executor(decode_pool,decode_wire,message,cache)
                            pipeline['media_decode_max_ms']=max(pipeline['media_decode_max_ms'],(time.perf_counter_ns()-decode_start)/1e6)
                            if media.get('error'):raise ValueError(media['error'])
                            if media.get('backend')!=backend or not controlling and (media.get('input') is not None or media.get('effective_action') is not None):raise ValueError('Wrong media backend')
                            received=time.perf_counter_ns()
                            media_latest={n:dict(media['sources'][n],pc_media_received_monotonic_ns=received,media_instance=media['instance']) for n in ('rgb','depth')}
                except asyncio.CancelledError:raise
                except Exception as exc:
                    media_latest={}
                    if not latest or latest.get('transport_profile')!='latest-preview-deferred-originals':await abort('媒体连接中断：'+str(exc))
                    pipeline['preview_error']=str(exc)
                    await asyncio.sleep(1)
        async def receive():
            nonlocal latest,pc_error,previous_instance,last_received_ns
            while True:
                try:
                    async with websockets.connect(gateway+'/ws/observations',max_size=8*1024*1024,proxy=None,compression=None,max_queue=4) as ws:
                        media_cache={}
                        clock=TransportClock();probes={};last_probe_ns=0;probe_count=0
                        async def probe():
                            nonlocal last_probe_ns
                            last_probe_ns=time.perf_counter_ns()
                            nonce=str(last_probe_ns);probes[nonce]=last_probe_ns
                            await ws.send(json.dumps(dict(operation='clock_probe',nonce=nonce)))
                        await probe()
                        async for message in ws:
                            frame=await asyncio.get_running_loop().run_in_executor(decode_pool,decode_wire,message,media_cache)
                            if 'clock_probe' in frame:
                                sent=probes.pop(frame['clock_probe'],None)
                                if sent is not None:clock.update(sent,time.perf_counter_ns(),frame['x5_monotonic_ns'])
                                probe_count+=1
                                if probe_count>=5 and clock.anchor:clock_ready.set()
                                if probe_count<5:await probe()
                                continue
                            if frame.get('error'):raise ValueError(frame['error'])
                            if frame.get('schema_version')!=(3 if controlling else 2) or frame.get('backend')!=backend:raise ValueError('Wrong observation backend/protocol')
                            if not controlling and (frame.get('input') is not None or frame.get('effective_action') is not None):raise ValueError('Action contamination')
                            native=frame
                            if native.get('transport_profile')!='latest-preview-deferred-originals':
                                if not media_latest:continue
                                selected=dict(robot=native['sources']['robot'])
                                selected.update({n:s for n,s in media_latest.items() if s['media_instance']==native['instance']})
                                frame=assemble(selected,native['stats'],native['instance'],native['observation']['frame_id'],native['observation']['gateway_monotonic_ns'])
                                if not frame['sources']['rgb'].get('jpeg'):frame['observation']['valid']=False
                            else:
                                # Native source metadata drives freshness and archive associations;
                                # the independently compressed preview has its own identity/age.
                                frame['preview']={}
                                for n,media in media_latest.items():
                                    if media['media_instance']==frame['instance']:
                                        bounds=clock.bounds_ms(media['x5_receive_monotonic_ns'],time.perf_counter_ns())
                                        frame['preview'][n]=dict(device_frame_id=media.get('device_frame_id'),
                                            width=media.get('preview_width'),height=media.get('preview_height'),
                                            age_upper_ms=bounds['age_upper_ms'] if bounds else None,
                                            valid=bool(bounds and bounds['age_upper_ms']<=500),
                                            jpeg=media.get('preview_jpeg'))
                            frame['pc_received_monotonic_ns']=time.perf_counter_ns()
                            frame['stats']['pc_tcp_latency']=dict(pc_tcp_profile or {})
                            if clock.anchor is None:
                                # Do not label the initial uncalibrated transport
                                # as fresh. The watchdog/start gate remain closed.
                                continue
                            clock.apply(frame,frame['pc_received_monotonic_ns'])
                            if frame['pc_received_monotonic_ns']-last_probe_ns>5_000_000_000:await probe()
                            last_received_ns=frame['pc_received_monotonic_ns']
                            if previous_instance is not None and frame['instance']!=previous_instance:await abort('X5网关实例已改变')
                            previous_instance=frame['instance']
                            try:incoming.put_nowait(frame)
                            except asyncio.QueueFull:
                                await abort('PC观测处理队列溢出');raise ValueError('Processing queue overflow')
                except asyncio.CancelledError:raise
                except Exception as exc:
                    await abort('观测连接中断：'+str(exc));latest=None
                    broadcast(dict(error=pc_error,status=dict(armed=False,reason=pc_error)))
                    await asyncio.sleep(1)
        async def process():
            nonlocal latest,pc_error,first_valid
            while True:
                frame=await incoming.get()
                process_start=time.perf_counter_ns()
                now=time.perf_counter_ns()
                queued_ms=(now-frame['pc_received_monotonic_ns'])/1e6
                pipeline['queue_max_ms']=max(pipeline['queue_max_ms'],queued_ms)
                frame['pc_pipeline']=dict(pipeline)
                frame['stats']['pc_model_renderer']=dict(renderer_report)
                for name,limit in (('robot',500),('rgb',500),('depth',500)):
                    source=frame['sources'][name]
                    age=source.get('pc_age_upper_ms')
                    if age is not None:
                        source['pc_age_upper_ms']=age+queued_ms
                        if source['pc_age_upper_ms']>limit:source.update(valid=False,reason='PC处理队列中的观测已过期')
                frame['observation']['valid']=frame['observation']['valid'] and frame['sources']['robot']['valid'] and frame['sources']['rgb']['valid']
                frame['pc_sampled_monotonic_ns']=now
                latest=frame
                if frame['observation']['valid'] and not first_valid:
                    pc_error=None;first_valid=True
                if not frame['observation']['valid'] or frame.get('stats',{}).get('fatal'):
                    detail={n:{k:frame['sources'][n].get(k) for k in ('valid','reason','age_ms','pc_age_upper_ms')} for n in ('robot','rgb')}
                    await abort('机械臂或RGB无有效新观测：'+str(frame.get('stats',{}).get('fatal') or detail))
                q=frame['observation'].get('joints_rad')
                if q:
                    def link6_pose():
                        model.set_state(q,0)
                        return model.pose()
                    position,quat=await asyncio.get_running_loop().run_in_executor(kinematics_pool,link6_pose)
                    frame['observation'].update(model_link6_position_m=position.tolist(),model_link6_quaternion_xyzw=quat.tolist())
                    if controlling:
                        decoded=frame['sources']['robot']['decoded']
                        frame['observation'].update(tcp_position_m=decoded['controller_position_m'],tcp_quaternion_xyzw=decoded['controller_quaternion_xyzw'],pose_reference='controller User0 Tool0 flange origin')
                    else:frame['observation'].update(tcp_position_m=position.tolist(),tcp_quaternion_xyzw=quat.tolist(),pose_reference='URDF Link6 origin, model illustration')
                async with gate:
                    if recorder.active and (not recorder.deferred or frame.get('archive_session')==recorder.directory.name):
                        try:await asyncio.to_thread(recorder.sample,frame)
                        except Exception as exc:
                            await stop_control()
                            recorder.incomplete=True
                            try:
                                try:await asyncio.to_thread(recorder.retain_failure,frame,str(exc))
                                except Exception:pass
                                await asyncio.to_thread(recorder.finish,'aborted','录制失败：'+str(exc))
                                schedule_media(recorder.instance or frame['instance'])
                            except Exception:pass
                            pc_error='录制失败：'+str(exc)
                try:
                    frame['images']=dict(cached_images,wrist_jpeg=frame['sources']['rgb'].get('jpeg'))
                    if frame.get('transport_profile')=='latest-preview-deferred-originals':
                        frame['images'].update(wrist_jpeg=frame.get('preview',{}).get('rgb',{}).get('jpeg'),depth_jpeg=frame.get('preview',{}).get('depth',{}).get('jpeg'))
                except Exception as exc:frame['model_error']=str(exc)
                frame['pc_processing_ms']=(time.perf_counter_ns()-process_start)/1e6
                frame['pc_queue_size']=incoming.qsize()
                if frame['observation']['frame_id']%2==0 or not frame['observation']['valid']:broadcast(frame)
                if audit_viewers:
                    audit={k:v for k,v in frame.items() if k not in ('images','preview')}
                    for queue in tuple(audit_viewers):
                        if queue.full():await abort('PC控制审计队列溢出');audit_viewers.discard(queue)
                        else:queue.put_nowait(audit)
        async def watchdog():
            while True:
                await asyncio.sleep(.1)
                if latest and last_received_ns is not None and time.perf_counter_ns()-last_received_ns>500_000_000:
                    await abort('PC观测传输超过500ms未更新')
                    stale=dict(latest,observation=dict(latest['observation'],valid=False),error=pc_error)
                    broadcast(stale)
        async def monitor_loop():
            while True:
                expected=time.perf_counter_ns()+50_000_000
                await asyncio.sleep(.05)
                pipeline['event_loop_lag_max_ms']=max(pipeline['event_loop_lag_max_ms'],(time.perf_counter_ns()-expected)/1e6)
        tasks=[asyncio.create_task(fn()) for fn in (receive_media,receive,process,watchdog,preview,monitor_loop)]
        try:yield
        finally:
            await abort('观测工作台退出')
            for task in tuple(finalizers):task.cancel()
            await asyncio.gather(*tuple(finalizers),return_exceptions=True)
            for task in tasks:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
            await asyncio.get_running_loop().run_in_executor(render_pool,preview_model.close)
            model.close()
            render_pool.shutdown()
            decode_pool.shutdown()
            kinematics_pool.shutdown()

    app=FastAPI(lifespan=lifespan)
    @app.middleware('http')
    async def origin(request:Request,call_next):
        value=request.headers.get('origin')
        if value and urlparse(value).hostname not in {'localhost','127.0.0.1','::1'}:return JSONResponse({'error':'Local origin required'},status_code=403)
        return await call_next(request)
    @app.get('/api/state')
    def state():
        fresh=bool(latest and time.perf_counter_ns()-latest['pc_received_monotonic_ns']<=500_000_000 and latest['observation']['valid'])
        return dict(backend=backend,hardware_connected=fresh,gateway_ready=fresh,recording=recorder.state(),
                    export={'state':'disabled'},capabilities={'control':controlling,'operator_control':controlling and control_role=='operator','controller_role':control_role if controlling else None},
                    control_stage=latest.get('control_stage') if latest else None,status=latest.get('status') if latest else None,error=pc_error,preview_discarded=preview_discarded,exiting=exiting)
    def stopped_for_exit():
        # The published stop proof must still belong to fresh actual feedback.
        # Closing resources never changes enable, Tool0 or any other parameter.
        if not latest or time.perf_counter_ns()-latest['pc_received_monotonic_ns']>100_000_000:return False
        s=latest.get('status') or {};robot=latest.get('sources',{}).get('robot',{});d=robot.get('decoded') or {}
        if s.get('armed') or s.get('pending_command_id') is not None or not s.get('stop_acknowledged') or not s.get('stopped_feedback'):return False
        if not robot.get('valid') or d.get('robot_mode') not in (4,5) or d.get('run_queued_command'):return False
        actual=d.get('q_actual_deg',[]);target=d.get('q_target_deg',[])
        return len(actual)==len(target)==6 and all(np.isfinite(a) and np.isfinite(b) and abs(a-b)<=.05 for a,b in zip(actual,target))
    @app.post('/api/workbench/exit')
    async def exit_workbench(request:Request):
        nonlocal exiting,exit_confirmed
        supplied=request.headers.get('X-E6-Exit-Token','')
        if not exit_token or not secrets.compare_digest(supplied,exit_token):raise HTTPException(403,'Owned workbench exit token required')
        if exit_callback is None:raise HTTPException(409,'Managed exit unavailable')
        data=await request.json()
        if not latest or data.get('instance')!=latest.get('instance') or data.get('backend')!=backend:raise HTTPException(409,'Workbench instance/backend changed')
        if exiting:raise HTTPException(409,'Workbench exit already requested')
        exiting=True
        try:
            if controlling and latest.get('control_stage') not in ('shadow','live'):raise ValueError('Unknown control stage; workbench retained')
            if controlling and latest.get('control_stage')=='live':
                if controller:await controller.rpc(dict(operation='stop'))
                elif not stopped_for_exit():
                    # Use the existing guardian and its sole Dashboard link;
                    # never open a second E6 connection or initialize CAN.
                    from .control_transport import StreamingGatewayConnection
                    async with websockets.connect(control_gateway+'/ws/control',proxy=None,compression=None,
                            additional_headers={'X-E6-Control-Token':control_token,'X-E6-Control-Role':control_role},max_size=65536) as ws:
                        hello=json.loads(await ws.recv())
                        if not hello.get('ok'):raise ValueError('Stop controller unavailable')
                        async def ignore_reply(reply):pass
                        temporary=StreamingGatewayConnection(ws,hello,ignore_reply)
                        try:await temporary.rpc(dict(operation='stop'))
                        finally:await temporary.close()
                until=time.monotonic()+2
                while not stopped_for_exit():
                    if time.monotonic()>=until:raise ValueError('Stop confirmation or fresh stationary feedback unavailable; workbench retained')
                    await asyncio.sleep(.02)
            exit_confirmed=True
            # Return the acknowledgement before Uvicorn starts normal lifespan
            # cleanup. cli.finally then closes the owned SSH runtime and route.
            asyncio.get_running_loop().call_later(.1,exit_callback)
            return dict(exiting=True,backend=backend,instance=latest['instance'],
                        stop_confirmed=controlling and latest.get('control_stage')=='live',
                        enable_changed=False)
        except Exception as exc:
            exiting=False
            raise HTTPException(503,str(exc)) from exc
    @app.websocket('/ws/control')
    async def rejected(ws:WebSocket):
        nonlocal controller,owner
        origin=ws.headers.get('origin')
        if origin and urlparse(origin).hostname not in {'localhost','127.0.0.1','::1'}:await ws.close(1008);return
        await ws.accept()
        if exiting:
            await ws.send_json(dict(ok=False,error='工作台正在退出'));await ws.close(1008);return
        if not controlling:
            await ws.send_json(dict(ok=False,error='观测后端没有动作通道'));await ws.close(1008);return
        if control_role=='harness' and ws.headers.get('X-E6-Control-Token')!=control_token:
            await ws.send_json(dict(ok=False,error='自动验收期间不开放网页控制'));await ws.close(1008);return
        if owner is not None:
            await ws.send_json(dict(ok=False,error='已有控制者'));await ws.close(1008);return
        import secrets
        from .control_transport import StreamingGatewayConnection
        owner=secrets.token_urlsafe(12);owned=owner
        try:
            async with websockets.connect(control_gateway+'/ws/control',proxy=None,compression=None,additional_headers={'X-E6-Control-Token':control_token,'X-E6-Control-Role':control_role},max_size=65536) as upstream:
                hello=json.loads(await upstream.recv())
                if not hello.get('ok'):await ws.send_json(hello);return
                controller=StreamingGatewayConnection(upstream,hello,ws.send_json);await ws.send_json(hello)
                while True:
                    data=await ws.receive_json()
                    if not isinstance(data,dict):raise ValueError('Object command required')
                    if exiting and data.get('operation')!='stop':raise ValueError('工作台正在退出，控制已暂停')
                    if data.get('operation') not in ('input','arm','stop','enable','initialize-gripper','diagnose-kinematics'):raise ValueError('No command passthrough')
                    if recorder.pending and data.get('operation')!='stop':
                        await stop_control();await ws.send_json(dict(ok=False,error='媒体回传中，控制暂停',status=controller.status));continue
                    await controller.send(data)
        except (WebSocketDisconnect,RuntimeError):pass
        except Exception as exc:
            try:await ws.send_json(dict(ok=False,error=str(exc)))
            except Exception:pass
        finally:
            await stop_control()
            if owner==owned:
                if controller:await controller.close()
                owner=None;controller=None
    @app.post('/api/control')
    def reject_control():raise HTTPException(403,'观测后端拒绝动作和参数写入请求')
    @app.websocket('/ws/state')
    async def state_ws(ws:WebSocket):
        origin=ws.headers.get('origin')
        if origin and urlparse(origin).hostname not in {'localhost','127.0.0.1','::1'}:
            await ws.close(1008);return
        await ws.accept();queue=asyncio.Queue(maxsize=1);viewers.add(queue)
        try:
            if latest:broadcast(latest)
            while True:await ws.send_json(await queue.get())
        except (WebSocketDisconnect,RuntimeError):pass
        finally:viewers.discard(queue)
    @app.websocket('/ws/control-audit')
    async def audit_ws(ws:WebSocket):
        if not controlling or ws.headers.get('X-E6-Control-Token')!=control_token:
            await ws.close(1008);return
        await ws.accept();queue=asyncio.Queue(maxsize=64);audit_viewers.add(queue)
        try:
            while True:await ws.send_json(await queue.get())
        except (WebSocketDisconnect,RuntimeError):pass
        finally:audit_viewers.discard(queue)
    @app.post('/api/record/start')
    async def start(request:Request):
        nonlocal pc_error
        if exiting:raise HTTPException(409,'工作台正在退出')
        data=await request.json()
        async with gate:
            if not latest or not latest['observation']['valid'] or time.perf_counter_ns()-latest['pc_received_monotonic_ns']>250_000_000:raise HTTPException(409,'有效观测尚未就绪')
            if latest.get('stats',{}).get('fatal'):raise HTTPException(409,'网关采集异常，需要重新启动')
            if controlling and (latest.get('status') or {}).get('faulted'):raise HTTPException(409,'控制故障后需要回中并显式重新准备，不能直接续录')
            try:
                await asyncio.to_thread(recorder.start,data.get('task'),dict(instance=latest['instance'],control_stage=latest.get('control_stage'),control_limits=(latest.get('status') or {}).get('limits'),device_configuration=latest['stats'],media_storage='x5-spool-then-verified-pc' if latest.get('capabilities',{}).get('deferred_media') else 'inline-native'))
                pc_error=None
            except ValueError as exc:raise HTTPException(409,str(exc)) from exc
        if recorder.deferred:
            try:await begin_archive(archive_gateway,latest['instance'],recorder.directory.name)
            except Exception as exc:
                await abort('X5原始媒体暂存启动失败：'+str(exc))
                raise HTTPException(409,'X5原始媒体暂存启动失败') from exc
        return recorder.state()
    @app.post('/api/record/finish')
    async def finish(request:Request):
        data=await request.json()
        await stop_control()
        async with gate:
            try:
                await asyncio.to_thread(recorder.finish,data.get('result','success' if controlling else 'observed'))
                schedule_media(recorder.instance or previous_instance)
            except ValueError as exc:raise HTTPException(409,str(exc)) from exc
        return recorder.state()
    def episode(name):
        path=(raw_root/name).resolve()
        if path.parent!=raw_root or not (path/'metadata.json').is_file():raise HTTPException(404)
        return path
    @app.get('/api/episodes')
    def episodes():
        result=[]
        for path in sorted(raw_root.glob('*'),reverse=True):
            if not (path/'metadata.json').exists() or (recorder.active and path==recorder.directory):continue
            meta=json.loads((path/'metadata.json').read_text(encoding='utf-8'));summary=read_summary(path)
            result.append(dict(id=path.name,task=meta['task'],dataset_kind=dataset_kind,**{k:v for k,v in summary.items() if k!='dataset_kind'}))
        return result
    def read_summary(path):
        try:return json.loads((path/'summary.json').read_text(encoding='utf-8'))
        except (FileNotFoundError,ValueError):
            samples=0
            if (path/'samples.jsonl').exists():
                with (path/'samples.jsonl').open(encoding='utf-8') as stream:samples=sum(1 for line in stream if line.strip())
            return dict(result='aborted',complete=False,samples=samples,reason='结束摘要缺失或不完整')
    @app.get('/api/episodes/{name}')
    def detail(name:str,offset:int=0,limit:int=100):
        if offset<0 or not 1<=limit<=200:raise HTTPException(400,'Invalid observation page')
        path=episode(name);summary=read_summary(path);rows=[]
        with (path/'samples.jsonl').open(encoding='utf-8') as stream:
            for index,line in enumerate(stream):
                if index<offset:continue
                if len(rows)>=limit:break
                try:rows.append(json.loads(line))
                except ValueError:break
        return dict(summary=summary,rows=rows)
    @app.get('/api/episodes/{name}/images/{filename}')
    def image(name:str,filename:str):
        path=(episode(name)/'images'/filename).resolve()
        if path.parent!=episode(name)/'images' or not path.exists():raise HTTPException(404)
        return FileResponse(path)
    @app.post('/api/export/{name}')
    def export(name:str):raise HTTPException(400,'本阶段真实记录禁止通过模拟LeRobot导出器导出')
    @app.get('/api/datasets')
    def datasets():return []
    dist=ROOT/'web'/'dist'
    if dist.exists():app.mount('/',StaticFiles(directory=dist,html=True),name='web')
    return app
