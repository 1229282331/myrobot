"""Original episode journal built on the independent Audit and media spool.

No motion APIs live here. Control only enqueues events; recording, sealing and
bundling run in a worker. Raw source clocks and execution uncertainty survive.
"""
from datetime import datetime,timezone
import json
from pathlib import Path
import queue
import re
import threading
import time
import uuid
from .audit import Audit
from .camera_archive import atomic_json,read_atomic_json
from .episode_bundle import run_bundle
from .state_observation import observation_fields


def checked_id(value):
    if not isinstance(value,str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',value):
        raise ValueError('Invalid episode identifier')
    return value


def recording_context(value):
    """Optional, bounded operator annotations; never device configuration."""
    if value is None:return {}
    limits={'scene':500,'operator':80}
    if not isinstance(value,dict) or set(value)-limits.keys():
        raise ValueError('采集备注仅支持 scene 和 operator 文本字段')
    result={}
    for key,text in value.items():
        if not isinstance(text,str) or len(text)>limits[key]:
            raise ValueError(f'采集备注 {key} 需为最多{limits[key]}字的文本')
        if text.strip():result[key]=text.strip()
    return result


def media_capacity(camera):
    """Image-byte allowance, before archive compression; not disk free space."""
    if not isinstance(camera,dict):return None
    limit=camera.get('native_budget_bytes');used=camera.get('native_bytes')
    if type(limit) is not int or limit<=0 or type(used) is not int or used<0:return None
    return dict(limit_bytes=limit,used_bytes=used,remaining_bytes=max(0,limit-used),
                warning=used>=limit*.8,accounting='native_image_payload')


class Episodes:
    def __init__(self,root,camera_dir,reader,stage,*,audit_factory=Audit,camera_status_path=None):
        self.root=Path(root);self.camera_dir=Path(camera_dir);self.reader=reader
        self.camera_status_path=Path(camera_status_path) if camera_status_path is not None else self.camera_dir/'recording.status.json'
        self.stage=stage;self.factory=audit_factory
        self.commands=queue.Queue(4);self.lock=threading.RLock();self.halt=threading.Event()
        # Media acknowledgements can wait on disk or on each other, but must
        # never hold the status lock used by the independent control/Stop path.
        self.receipt_lock=threading.Lock()
        self.writer=None;self.directory=None;self.latest_state=None
        self.start_data=None
        self.shutdown_deadline=float('inf')
        self.value=dict(phase='idle',episode=None,error=None,samples=0,bundle_sha256=None)
        self.thread=threading.Thread(target=self._run,name='episode-journal',daemon=True)
        self.thread.start()

    def state(self):
        with self.lock:return dict(self.value)

    def request(self,operation,**data):
        with self.lock:
            phase=self.value['phase']
            if operation=='start':
                if phase not in ('idle','complete','aborted'):raise ValueError('当前片段尚未完成或等待回传')
                task=data.get('task','')
                if not isinstance(task,str):raise ValueError('任务说明需为文本')
                task=task.strip()
                if not 1<=len(task)<=500:raise ValueError('请输入1–500字的任务说明')
                data['research_context']=recording_context(data.get('research_context'))
                episode=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
                self.directory=None;self.start_data=data
                self.value=dict(phase='starting',episode=episode,error=None,samples=0,bundle_sha256=None)
                data['episode']=episode
                data['task']=task
            elif operation=='finish':
                if phase not in ('starting','capturing'):raise ValueError('没有正在录制的片段')
                if data.get('result') not in ('success','failure','aborted'):raise ValueError('未知片段结果')
                self.value['phase']='sealing'
            else:raise ValueError('Unknown recording operation')
            try:self.commands.put_nowait((operation,data))
            except queue.Full:
                self.value.update(phase='aborted',error='episode_command_overflow')
                raise

    def abort(self,reason):
        if self.state()['phase'] in ('starting','capturing'):
            try:self.request('finish',result='aborted',reason=reason)
            except (ValueError,queue.Full):pass

    def put(self,event):
        with self.lock:
            if event['event']=='cycle':self.latest_state=event['state']
            writer=self.writer
            if writer is not None:
                writer.put(event)
                if event['event']=='cycle':self.value['samples']+=1

    def _camera_request(self,op,episode):
        request_id=uuid.uuid4().hex
        atomic_json(self.camera_dir/'recording.request.json',dict(id=request_id,op=op,session=episode))
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            try:
                result=read_atomic_json(self.camera_status_path)
                if result.get('last_request')==request_id:
                    if result.get('error'):raise RuntimeError(result['error'])
                    return result
            except FileNotFoundError:pass
            time.sleep(.02)
        raise TimeoutError('相机原始记录任务未确认')

    def _start(self,data):
        episode=data['episode']
        directory=self.root/episode;directory.mkdir(parents=True,exist_ok=False)
        self.directory=directory
        metadata=dict(schema='e6-original-episode-v1',dataset_kind='hardware_demonstration' if self.stage=='live' else 'hardware_shadow',
            is_real_demonstration=self.stage=='live',control_stage=self.stage,episode=episode,task=data['task'],
            created_at=datetime.now(timezone.utc).isoformat(),x5_requested_ns=data['requested_ns'],
            quaternion_order='xyzw',units=dict(position='m',rotation='rad'),nominal_sample_hz=20,
            clock_note='Association uses X5 receive monotonic only. Device PTS and PC clocks are separate.',
            action_association_policy='sdk_available_v2',
            action_semantics='Original official ServoP/ServoJ and CAN sends with separate acknowledgements; no next-state substitution',
            initial_state=data.get('state'),export_allowed=False,
            research_context=data.get('research_context',{}))
        atomic_json(directory/'metadata.json',metadata)
        writer=self.factory(directory/'audit',self.reader)
        with self.lock:self.writer=writer
        camera=self._camera_request('start',episode)
        writer.put(dict(event='episode_started',at_ns=time.perf_counter_ns(),episode=episode))
        with self.lock:
            self.value['capacity']=media_capacity(camera)
            if self.value['phase']=='starting':self.value['phase']='capturing'

    def _finish(self,data):
        if self.directory is None or self.value['phase'] in ('transfer_pending','complete','aborted'):return
        episode=self.directory.name
        result=data['result'];reason=data.get('reason');errors=[]
        # A planned end retains Stop receipts and the settled feedback window.
        # A fault end is not upgraded to success because a later Stop succeeds.
        deadline=time.monotonic()+3
        while self.stage=='live' and result!='aborted' and time.monotonic()<deadline:
            state=observation_fields(self.latest_state) or {}
            if state.get('stationary') and not state.get('stopping'):break
            time.sleep(.02)
        if self.stage=='live' and result!='aborted':
            state=observation_fields(self.latest_state) or {}
            if not state.get('stationary') or state.get('stopping') or state.get('stop_uncertain'):
                errors.append('录制结束时停止结果未确定')
        try:media=self._camera_request('finish',episode)
        except Exception as exc:errors.append(str(exc));media=None
        with self.lock:writer=self.writer;self.writer=None
        if writer:
            writer.put(dict(event='episode_finished',at_ns=time.perf_counter_ns(),result=result,reason=reason,
                            final_state=self.latest_state))
            try:writer.close()
            except Exception as exc:errors.append(str(exc))
            if writer.error:errors.append(writer.error)
        summary=dict(episode=episode,result='aborted' if errors else result,reason=reason,
            errors=errors,samples=self.value['samples'],complete=not errors and result!='aborted',media=media,
            x5_finished_ns=time.perf_counter_ns(),phase='sealed',export_allowed=False,
            needs_pc_verification=True,action_labels='original execution events; not resampled')
        atomic_json(self.directory/'summary.json',summary)
        # Bundle even a failed or aborted recording when its retained files are readable.
        try:digest=self._bundle(episode)
        except Exception as exc:
            errors.append('打包失败: '+str(exc));digest=None
            summary.update(result='aborted',complete=False,errors=errors)
            atomic_json(self.directory/'summary.json',summary)
        atomic_json(self.directory/'transfer.json',dict(episode=episode,bundle_sha256=digest,
                    phase='pending' if digest else 'failed'))
        with self.lock:self.value.update(phase='transfer_pending' if digest else 'aborted',
            capacity=media_capacity(media) if media else self.value.get('capacity'),
            error='; '.join([str(x) for x in [reason,*errors] if x]) if result=='aborted' or errors else None,
            result=summary['result'],bundle_sha256=digest)

    def _bundle(self,episode):
        directory=self.root/checked_id(episode);media=self.camera_dir/'media'/episode
        return run_bundle(directory,media,shutdown_deadline=lambda:self.shutdown_deadline)

    def transferred(self,episode,digest):
        with self.receipt_lock:
            checked_id(episode)
            saved=read_atomic_json(self.root/episode/'transfer.json')
            if not digest or digest!=saved['bundle_sha256']:raise ValueError('回传校验标识不一致')
            atomic_json(self.root/episode/'transfer.json',dict(episode=episode,bundle_sha256=digest,
                        phase='verified',x5_ack_ns=time.perf_counter_ns()))
            # A failed receipt leaves the recording pending. A delayed receipt
            # for an old episode cannot complete a newer recording.
            with self.lock:
                if episode==self.value['episode'] and self.value['phase']=='transfer_pending':
                    self.value['phase']='complete' if self.value.get('result')!='aborted' else 'aborted'
                return dict(self.value)

    def _run(self):
        while not self.halt.is_set() or not self.commands.empty():
            try:operation,data=self.commands.get(timeout=.1)
            except queue.Empty:
                with self.lock:writer=self.writer
                if writer and writer.error:self.abort('原始记录写盘失败: '+writer.error)
                elif writer and self.value['phase']=='capturing':
                    try:
                        camera=read_atomic_json(self.camera_status_path)
                        if camera.get('session')!=self.value['episode'] or camera.get('error'):
                            self.abort('相机记录异常: '+str(camera.get('error') or 'session mismatch'))
                        else:
                            with self.lock:self.value['capacity']=media_capacity(camera)
                    except Exception as exc:self.abort('相机记录状态不可读: '+str(exc))
                elif self.value['phase']=='idle' and 'capacity' not in self.value:
                    # One background read after startup; no filesystem work on
                    # state(), feedback, input, or Stop paths.
                    try:
                        camera=read_atomic_json(self.camera_status_path)
                        capacity=media_capacity(camera)
                        if capacity is not None:
                            capacity=media_capacity(dict(camera,native_bytes=0))
                            with self.lock:self.value['capacity']=capacity
                    except (OSError,ValueError):pass
                continue
            try:
                if operation=='start':self._start(data)
                else:self._finish(data)
            except Exception as exc:
                # Even a partly-created journal keeps the failure and releases
                # its raw subscription. A queued finish cannot seal it twice.
                try:
                    if operation=='start' and self.directory is not None:
                        self._finish(dict(result='aborted',reason='初始化失败: '+str(exc)))
                    else:raise exc
                except Exception as cleanup:
                    with self.lock:writer=self.writer;self.writer=None
                    if writer:
                        try:writer.close()
                        except Exception:pass
                    with self.lock:self.value.update(phase='aborted',error=str(cleanup))
            finally:self.commands.task_done()

    def close(self):
        self.shutdown_deadline=time.monotonic()+10
        self.abort('服务退出');self.halt.set();self.thread.join(30)
        if self.thread.is_alive():raise RuntimeError('Episode journal did not exit')
