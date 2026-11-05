"""Managed exit uses the owned guardian, then preserves enable during cleanup."""
import asyncio
import json
import time
import threading
from unittest.mock import patch

from fastapi.testclient import TestClient
import pytest

from e6_teleop.observation_service import make_app
from test_observation import Connection


class Streams:
    def __init__(self,backend='hardware-control',stage='live',violation=None):
        self.backend,self.stage,self.violation=backend,stage,violation
        self.commands=[]
        self.stop_received=threading.Event()
        self.status=dict(armed=False,pending_command_id=None,stop_acknowledged=True,stopped_feedback=True)
        if violation=='armed':self.status['armed']=True
        if violation=='unconfirmed':self.status['stop_acknowledged']=False
        if violation=='moving':self.status['stopped_feedback']=False
        if violation=='race':self.status['stopped_feedback']=False

    def connect(self,url,**kwargs):
        parent=self
        if url.endswith('/ws/control'):
            class Control:
                def __init__(self):self.replies=asyncio.Queue()
                async def __aenter__(self):return self
                async def __aexit__(self,*args):pass
                async def recv(self):return json.dumps(dict(ok=True,status=dict(session='test-session')))
                async def send(self,raw):
                    command=json.loads(raw);parent.commands.append(command['operation'])
                    if parent.violation=='armed':parent.status.update(armed=False,stop_acknowledged=True,stopped_feedback=True)
                    if parent.violation=='race':
                        asyncio.get_running_loop().call_later(.12,lambda:parent.status.update(stopped_feedback=True))
                    parent.stop_received.set()
                    await self.replies.put(json.dumps(dict(ok=True,status=dict(session='test-session'),request_id=command['request_id'])))
                def __aiter__(self):return self
                async def __anext__(self):return await self.replies.get()
                async def close(self):pass
            return Control()
        class Feed(Connection):
            async def messages(self):
                async for raw in super().messages():
                    f=json.loads(raw)
                    if 'observation' in f:
                        f.update(backend=parent.backend,schema_version=3 if parent.backend=='hardware-control' else 2,
                                 transport_profile='latest-preview-deferred-originals')
                        if parent.backend=='hardware-control':f.update(control_stage=parent.stage,status=dict(parent.status))
                        d=f['sources']['robot']['decoded']
                        d.update(robot_mode=5,enable_status=1,run_queued_command=0,q_target_deg=list(d['q_actual_deg']))
                        if parent.violation=='nan':d['q_actual_deg'][0]=float('nan')
                        if parent.violation=='queue':d['run_queued_command']=1
                    yield json.dumps(f)
        return Feed('healthy')


def ready(client):
    until=time.monotonic()+2
    while not client.get('/api/state').json()['gateway_ready']:
        if time.monotonic()>until:raise AssertionError('Fixture stream not ready')
        time.sleep(.01)


def app(tmp_path,callback,backend='hardware-control'):
    return make_app(tmp_path,render=False,backend=backend,
                    control_gateway='ws://127.0.0.1:2' if backend=='hardware-control' else None,
                    control_token='control-test' if backend=='hardware-control' else None,
                    exit_token='exit-test',exit_callback=callback)


@pytest.mark.parametrize('backend,stage',[('hardware-control','live'),('hardware-control','shadow'),('hardware-observe',None)])
def test_confirmed_exit_no_motion_can_disable_or_extra_stop(tmp_path,backend,stage):
    streams=Streams(backend,stage);closed=[]
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=streams.connect):
        with TestClient(app(tmp_path,lambda:closed.append(True),backend)) as c:
            ready(c)
            reply=c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json={'instance':'one','backend':backend})
            assert reply.status_code==200 and reply.json()['enable_changed'] is False
            assert reply.json()['stop_confirmed']==(backend=='hardware-control' and stage=='live')
            assert c.post('/api/record/start',json={'task':'late'}).status_code==409
            with c.websocket_connect('/ws/control') as ws:assert not ws.receive_json()['ok']
            time.sleep(.15)
            assert closed==[True] and streams.commands==[]


def test_active_exit_requests_one_owned_stop_then_waits_for_actual_proof(tmp_path):
    streams=Streams(violation='armed');closed=[]
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=streams.connect):
        with TestClient(app(tmp_path,lambda:closed.append(True))) as c:
            ready(c)
            reply=c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json={'instance':'one','backend':'hardware-control'})
            assert reply.status_code==200
            time.sleep(.15)
            assert closed==[True] and streams.commands==['stop']


@pytest.mark.parametrize('violation',['unconfirmed','moving','queue','nan'])
def test_missing_stop_or_actual_proof_retains_workbench(tmp_path,violation):
    streams=Streams(violation=violation);closed=[]
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=streams.connect):
        with TestClient(app(tmp_path,lambda:closed.append(True))) as c:
            ready(c)
            reply=c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json={'instance':'one','backend':'hardware-control'})
            assert reply.status_code==503
            assert not c.get('/api/state').json()['exiting']
            assert not closed and streams.commands==['stop']


def test_exit_requires_private_token_local_origin_and_current_instance(tmp_path):
    streams=Streams();closed=[]
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=streams.connect):
        with TestClient(app(tmp_path,lambda:closed.append(True))) as c:
            ready(c)
            body={'instance':'one','backend':'hardware-control'}
            assert c.post('/api/workbench/exit',json=body).status_code==403
            assert c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test','Origin':'https://external.example'},json=body).status_code==403
            assert c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json=dict(body,instance='old')).status_code==409
            assert c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json=dict(body,backend='simulation')).status_code==409
            assert not closed and not streams.commands


def test_input_disconnect_during_managed_exit_does_not_request_stop_twice(tmp_path):
    streams=Streams(violation='race');closed=[]
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=streams.connect):
        with TestClient(app(tmp_path,lambda:closed.append(True))) as c:
            ready(c)
            with c.websocket_connect('/ws/control',headers={'X-E6-Control-Token':'control-test'}) as ws:
                assert ws.receive_json()['ok']
                def late_input():
                    assert streams.stop_received.wait(1)
                    time.sleep(.01)
                    ws.send_json({'operation':'input'})
                sender=threading.Thread(target=late_input);sender.start()
                reply=c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json={'instance':'one','backend':'hardware-control'})
                sender.join(1)
                assert reply.status_code==200 and streams.commands==['stop']
                time.sleep(.15)
                assert closed==[True]


def test_unknown_control_stage_cannot_bypass_live_stop_gate(tmp_path):
    streams=Streams(stage='unknown');closed=[]
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=streams.connect):
        with TestClient(app(tmp_path,lambda:closed.append(True))) as c:
            ready(c)
            reply=c.post('/api/workbench/exit',headers={'X-E6-Exit-Token':'exit-test'},json={'instance':'one','backend':'hardware-control'})
            assert reply.status_code==503
            assert not closed and not streams.commands
