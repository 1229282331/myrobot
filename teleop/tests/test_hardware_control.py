import base64
import copy
import json
import math
from pathlib import Path
import struct
import sys
import time
from unittest.mock import patch
import numpy as np
import pytest
from fastapi.testclient import TestClient
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'device_gateway'))
from e6_observer.control_core import ControlCore,ControlLimits
from e6_observer.control_math import angle,controller_pose,integrate,pose
from e6_observer.protocol import decode
from e6_teleop.observation_recording import ObservationRecorder
from e6_teleop.observation_service import make_app
from e6_teleop.exporter import load_episode
from test_observation import frame,packet

class Clock:
    def __init__(self):self.ns=10_000_000_000
    def __call__(self):return self.ns
    def step(self,s=.05):self.ns+=round(s*1e9)

class Device:
    def __init__(self,clock):
        self.clock=clock;self.anchor=np.radians([-66,-60,105,-90,90,9]);self.q=self.anchor.copy()
        self.target=self.q.copy();self.identity=0;self.servo_count=0;self.stop_count=0;self.grip_count=0;self.ik_error=False;self.late=False
    def fk(self,q,deadline):
        delta=np.radians(q)-self.anchor
        return [*(np.array([0,-200,400])+delta[:3]*400),*np.degrees(delta[3:])]
    def ik(self,p,q,deadline):
        if self.ik_error:raise ValueError('unreachable')
        result=self.anchor+np.r_[(np.asarray(p[:3])-[0,-200,400])/400,np.radians(p[3:])]
        return np.degrees(result).tolist()
    def servo(self,q,deadline):
        self.servo_count+=1;self.identity+=1;self.q=np.radians(q);self.target=self.q.copy()
        if self.late:self.clock.step(.076)
        return self.identity
    def stop(self):self.stop_count+=1
    def grip(self,target):self.grip_count+=1
    def state(self,mode=5):
        return dict(x5_receive_monotonic_ns=self.clock(),decoded=dict(joints_rad=self.q.tolist(),q_target_deg=np.degrees(self.target).tolist(),
            robot_mode=mode,enable_status=int(mode!=4),error_status=0,user_index=0,tool_index=0,
            controller_pose_raw_mm_deg=self.fk(np.degrees(self.q),0),tool_configuration_raw_mm_deg=[0]*6,
            collision_state=0,pause_command_flag=0,run_queued_command=0,current_command_id=self.identity))

def setup(stage='shadow'):
    clock=Clock();device=Device(clock);core=ControlCore(device,stage,clock=clock);core.connect()
    return clock,device,core

def send(core,device,**kwargs):
    data=dict(session=core.owner,operation='input',lease=core.lease,seq=core.seq+1,source='keyboard',identity='synthetic',connected=True,focused=True,deadman=False,fine=False,stop=False,neutral=True,frame='base',mode='translation',twist=[0]*6,gripper_rate=0,raw={})
    data.update(kwargs);return core.command(data,device.state(),core.clock())

def arm(core,device):
    if core.stage=='live':
        core.gripper_initialized=True;core.grip_target=.2
        for _ in range(101):
            send(core,device)
            core.tick(device.state(),{'rgb':{'x5_receive_monotonic_ns':core.clock()}});core.clock.step()
    send(core,device)
    return core.command(dict(session=core.owner,operation='arm'),device.state(),core.clock())

def tick(clock,device,core):
    clock.step();return core.tick(device.state(),{'rgb':{'x5_receive_monotonic_ns':clock()}})

def test_shadow_same_math_and_zero_device_writes():
    c,d,g=setup();arm(g,d)
    for _ in range(20):
        send(g,d,neutral=False,deadman=True,twist=[.002,0,0,0,0,0]);r=tick(c,d,g)
        assert r['candidate_action'] and r['effective_action'] is None
    assert g.target_p[0]>0 and d.servo_count==d.grip_count==d.stop_count==0
    np.testing.assert_array_equal(d.q,d.anchor)


def test_successful_prepare_keeps_current_neutral_lease_but_stop_revokes_it():
    c,d,g=setup();send(g,d);token=g.lease
    g.command(dict(operation='arm',session=g.owner),d.state(),c())
    assert g.lease==token and g.armed
    # Neutral heartbeats remain valid while the preparation ACK is in flight.
    for _ in range(7):c.step();send(g,d)
    assert g.lease==token and g.armed and d.servo_count==0
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    assert g.lease!=token and not g.armed
    with pytest.raises(ValueError,match='租约'):
        g.command(dict(operation='input',session=g.owner,lease=token,seq=g.seq+1,twist=[0]*6,gripper_rate=0),d.state(),c())

def test_live_in_progress_target_skips_integration_and_never_queues():
    c,d,g=setup('live');arm(g,d)
    send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0]);first=tick(c,d,g)
    assert d.servo_count==1 and g.pending
    goal=g.candidate_q.copy();p=g.target_p.copy()
    c.step();send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    state=d.state(mode=7);state['decoded']['run_queued_command']=1
    state['decoded']['q_target_deg'][0]-=.051
    r=g.tick(state,{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert g.armed and d.servo_count==1 and r['effective_action'] is None
    assert r['skipped_control_period_reason']=='awaiting_previous_completion'
    np.testing.assert_array_equal(goal,g.candidate_q);np.testing.assert_array_equal(p,g.target_p)
    # Completion permits one fresh increment, never a compensating double step.
    c.step();send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    r=g.tick(d.state(),{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert g.armed and d.servo_count==2 and r['effective_action']
    assert np.linalg.norm(g.target_p-p)<=.001*.05

def test_live_uncompleted_target_still_faults_at_completion_deadline():
    c,d,g=setup('live');arm(g,d);send(g,d,deadman=True);tick(c,d,g)
    for _ in range(4):
        c.step();send(g,d,deadman=True)
        state=d.state(mode=7);state['decoded']['run_queued_command']=1
        state['decoded']['q_target_deg'][0]-=.051
        g.tick(state,{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert not g.armed and g.faulted and d.servo_count==1 and d.stop_count==1


def test_first_device_fault_survives_old_lease_rejections_and_requires_preparation():
    c,d,g=setup();arm(g,d);old_lease=g.lease
    send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    c.step();state=d.state()
    def timeout(*args):raise TimeoutError('PositiveKin reply deadline')
    original=d.fk;d.fk=timeout
    # Keep a fresh feedback state; only the controller query fails.
    result=g.tick(state,{'rgb':{'x5_receive_monotonic_ns':c()}})
    d.fk=original
    first=g.status()['first_fault']
    assert first['reason']=='PositiveKin reply deadline'
    assert not result['status']['armed'] and g.lease!=old_lease
    with pytest.raises(ValueError):send(g,d,lease=old_lease)
    g.stop('operator stopped after failure')
    assert g.status()['first_fault']==first
    # A neutral message or reconnect alone must not hide the cause.
    send(g,d);assert g.status()['first_fault']==first
    arm(g,d);assert g.status()['first_fault'] is None
    assert d.servo_count==d.grip_count==d.stop_count==0

def test_live_action_labels_are_accepted_targets_not_next_feedback():
    c,d,g=setup('live');arm(g,d)
    send(g,d,neutral=False,deadman=True,twist=[.002,0,0,0,0,0]);before=d.q.copy();r=tick(c,d,g)
    a=r['effective_action'];assert a['acceptance']=='controller_acknowledged_not_completion'
    np.testing.assert_array_equal(r['pre_action_robot']['decoded']['joints_rad'],before)
    assert a['joints_rad'][0]>before[0] and a['gateway_sent_ns']>=r['control_tick_monotonic_ns']
    assert d.servo_count==1 and d.grip_count==1
    send(g,d);assert g.armed and d.stop_count==1
    held=d.q.copy();tick(c,d,g);np.testing.assert_array_equal(d.q,held)

@pytest.mark.parametrize('fault',['duplicate','order','lease','focused','connected','stop','change','mode','nan','false-neutral','watchdog','robot-stale','rgb-stale','collision','pause','disabled','tool','joint-bound','unreachable','late','radius','rotation','velocity','acceleration'])
def test_fault_stops_and_never_replays(fault):
    c,d,g=setup('live');arm(g,d);send(g,d,neutral=False,deadman=True,twist=[.002,0,0,0,0,0]);tick(c,d,g)
    count=d.servo_count
    if fault in ('duplicate','order','lease','focused','connected','stop','change','mode','nan','false-neutral'):
        kwargs={'duplicate':{'seq':g.seq},'order':{'seq':g.seq-1},'lease':{'lease':'old'},'focused':{'focused':False},'connected':{'connected':False},'stop':{'stop':True},'change':{'frame':'tool','deadman':True,'neutral':False},'mode':{'mode':'rotation','twist':[.001,0,0,0,0,0]},'nan':{'twist':[float('nan'),0,0,0,0,0]},'false-neutral':{'neutral':True,'twist':[.001,0,0,0,0,0]}}[fault]
        try:send(g,d,**kwargs)
        except ValueError:pass
    else:
        send(g,d,neutral=False,deadman=True,twist=[.002,0,0,0,0,0]);s=d.state();media={'rgb':{'x5_receive_monotonic_ns':c()}}
        if fault=='watchdog':c.step(.3)
        elif fault=='robot-stale':s['x5_receive_monotonic_ns']-=101_000_000
        elif fault=='rgb-stale':media['rgb']['x5_receive_monotonic_ns']-=501_000_000
        elif fault=='collision':s['decoded']['collision_state']=1
        elif fault=='pause':s['decoded'].update(pause_command_flag=1,robot_mode=10)
        elif fault=='disabled':s['decoded']['enable_status']=0
        elif fault=='tool':s['decoded']['tool_configuration_raw_mm_deg'][2]=7
        elif fault=='joint-bound':s['decoded']['joints_rad'][1]=3
        elif fault=='unreachable':d.ik_error=True
        elif fault=='late':d.late=True
        elif fault=='radius':g.target_p[0]=.051
        elif fault=='rotation':g.target_r=pose([0,0,0,0,0,11])[1]
        elif fault in ('velocity','acceleration'):
            old=d.ik
            def excessive(p,q,deadline):
                result=old(p,q,deadline);result[0]+=.12 if fault=='velocity' else .025
                return result
            d.ik=excessive
            # Keep FK residual under 1 mm while testing the independent joint gate.
            d.fk=lambda q,deadline:[*list(g.target_p*1000),0,0,0]
        c.step()
        if fault not in ('watchdog','robot-stale'):s['x5_receive_monotonic_ns']=c()
        g.tick(s,media)
    assert not g.armed,fault
    assert d.stop_count>=1
    assert d.servo_count<=count+(fault=='late')
    count=d.servo_count;tick(c,d,g);assert d.servo_count==count

def test_second_controller_and_reconnection_require_new_neutral():
    c,d,g=setup();arm(g,d)
    with pytest.raises(ValueError):g.connect()
    owner=g.owner;g.disconnect(owner);g.connect()
    with pytest.raises(ValueError):g.command(dict(session=g.owner,operation='arm'),d.state(),c())

def test_queue_id_is_not_completion_and_no_second_target():
    c,d,g=setup('live');arm(g,d);send(g,d,neutral=False,deadman=True,twist=[.001,0,0,0,0,0]);tick(c,d,g)
    s=d.state();s['decoded']['robot_mode']=7;s['decoded']['run_queued_command']=1
    s['decoded']['q_target_deg'][0]-=.051
    c.step();r=g.tick(s,{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert g.armed and g.pending and d.servo_count==1 and r['effective_action'] is None
    assert r['skipped_control_period_reason']=='awaiting_previous_completion'

def test_unconfirmed_stop_is_not_automatically_retransmitted():
    c,d,g=setup('live');arm(g,d);send(g,d,deadman=True);tick(c,d,g)
    def fail():d.stop_count+=1;raise TimeoutError('missing Stop reply')
    d.stop=fail;g.stop('first fault',fault=True)
    for _ in range(5):g.stop('watchdog still unsafe',force=True,fault=True)
    assert d.stop_count==1 and g.stop_acknowledged is False and not g.armed
    g.command(dict(session=g.owner,operation='stop'),d.state(),c())
    assert d.stop_count==2 and not g.armed and g.pending

@pytest.mark.parametrize('stage,mode,enable',[('shadow',4,0),('shadow',5,1),('live',5,1)])
def test_queue_pause_flag_one_only_allows_empty_idle_with_matching_target(stage,mode,enable):
    c,d,g=setup(stage);s=d.state();s['decoded'].update(robot_mode=mode,enable_status=enable,pause_command_flag=1)
    assert g.safety(s,c(),c()).shape==(6,)
    for changes in ({'run_queued_command':1},{'robot_mode':7},{'robot_mode':10},
                    {'q_target_deg':[1]*6},{'pause_command_flag':2}):
        bad=copy.deepcopy(s);bad['decoded'].update(changes)
        with pytest.raises(ValueError):g.safety(bad,c(),c())
    assert d.servo_count==d.stop_count==0

def test_tool_frame_and_quaternion_boundary():
    p,r=pose([0,0,0,0,0,179.9]);p2,r2=integrate(p,r,[.01,0,0],[0,0,math.radians(1)],.2,'tool')
    assert p2[0]<0;raw=controller_pose(p2,r2,[0,0,0,0,0,179.9])
    assert raw[5]==pytest.approx(180.1) and angle(pose(raw)[1],r2)<1e-7

def test_extended_feedback_pinned_offsets_and_v2_compatibility():
    raw=bytearray(packet());struct.pack_into('<6d',raw,192,1,2,3,4,5,6);raw[1038]=1;raw[1015]=1;struct.pack_into('<Q',raw,1112,123)
    assert 'collision_state' not in decode(raw)
    d=decode(raw,True);assert d['q_target_deg']==[1,2,3,4,5,6] and d['current_command_id']==123 and d['collision_state']==d['pause_command_flag']==1
    struct.pack_into('<d',raw,480,float('nan'))
    with pytest.raises(ValueError):decode(raw,True)

def hardware_frame(seq=0):
    f=frame(seq);f.update(schema_version=3,backend='hardware-control',control_stage='live')
    t=f['observation']['gateway_monotonic_ns'];f['observation'].update(control_tick_id=seq,control_tick_monotonic_ns=t)
    f['input']={'source':'keyboard','seq':seq,'twist':[.001,0,0,0,0,0]}
    f['effective_action']=dict(joints_rad=[0]*6,gripper=.2,gateway_sent_ns=t+1,gateway_accepted_ns=t+2,acceptance='controller_acknowledged_not_completion')
    return f

def test_hardware_recording_separate_kind_and_export_refusal(tmp_path):
    r=ObservationRecorder(tmp_path,'hardware_demonstration');name=r.start('empty signal test',{})
    r.sample(hardware_frame());r.sample(hardware_frame(1));r.finish('success')
    meta=json.loads((tmp_path/name/'metadata.json').read_text());assert meta['schema_version']==3 and meta['dataset_kind']=='hardware_demonstration'
    rows=[json.loads(v) for v in (tmp_path/name/'samples.jsonl').read_text(encoding='utf-8').splitlines()]
    assert rows[0]['effective_action']['joints_rad']==[0]*6 and rows[0]['observation']['joints_rad']!=[0]*6
    with pytest.raises(ValueError,match='simulation'):load_episode(tmp_path/name)

@pytest.mark.parametrize('fault',['post-image','early-action','shadow-action'])
def test_recording_rejects_fabricated_temporal_action_association(tmp_path,fault):
    r=ObservationRecorder(tmp_path,'hardware_demonstration');r.start('failure',{});f=hardware_frame()
    if fault=='post-image':f['sources']['rgb']['x5_receive_monotonic_ns']=f['observation']['control_tick_monotonic_ns']+1
    elif fault=='early-action':f['effective_action']['gateway_sent_ns']=0
    else:f['control_stage']='shadow'
    with pytest.raises(ValueError):r.sample(f)
    r.finish('aborted')

def test_pc_control_role_lock_no_action_passthrough(tmp_path):
    app=make_app(tmp_path,gateway='ws://127.0.0.1:1',render=False,backend='hardware-control',control_gateway='ws://127.0.0.1:2',control_token='owned-token')
    with TestClient(app) as c:
        with c.websocket_connect('/ws/control') as ws:assert not ws.receive_json()['ok']
        assert c.post('/api/control',json={'operation':'Load()'}).status_code==403
        assert not c.get('/api/state').json()['capabilities']['operator_control']

@pytest.mark.parametrize('fault',['missing','recorded-stall'])
def test_gripper_failure_keeps_arm_acceptance_and_blocks_future_can(fault):
    c,d,g=setup('live');arm(g,d)
    calls=[]
    def fail(target):
        calls.append(target)
        if fault=='recorded-stall':
            from omnipicker.tools.verify_can_control import decode_feedback,FRAME
            # Actual fault packet from the 2026-10-06 empty-load cycle.
            # Decode offline; no socket, CAN configuration or device access.
            decode_feedback(FRAME.pack(1,8,bytes.fromhex('0002460a17190000')),0,1,
                            bytes.fromhex('00440a2019190000'))
        raise TimeoutError('CAN missing')
    d.grip=fail
    send(g,d,deadman=True);r=tick(c,d,g)
    assert r['effective_action']['command_id']==1
    assert r['effective_action']['gripper'] is None
    assert r['effective_action']['gripper_acceptance']=='uncertain'
    assert r['effective_action']['stop_requested_after_acceptance']
    assert g.gripper_faulted and not g.armed and d.stop_count==1
    for _ in range(110):
        send(g,d);tick(c,d,g)
    assert len(calls)==1 and d.servo_count==1
    with pytest.raises(ValueError,match='夹爪存在故障'):
        g.command({'operation':'arm','session':g.owner},d.state(),c())
    assert len(calls)==1 and d.servo_count==1

@pytest.mark.parametrize('tick_id',[0,2])
def test_demonstration_tick_duplication_and_gap_abort(tmp_path,tick_id):
    r=ObservationRecorder(tmp_path,'hardware_demonstration');r.start('tick audit',{})
    r.sample(hardware_frame());f=hardware_frame(1);f['observation']['control_tick_id']=tick_id
    with pytest.raises(ValueError,match='tick'):r.sample(f)
    r.finish('aborted')

def test_async_enable_does_not_hold_control_lock_and_is_not_retried():
    import threading
    from e6_observer.control_guardian import Commissioner
    _,d,g=setup('live');lock=threading.RLock();entered=threading.Event();release=threading.Event()
    class Link:
        cancel=threading.Event()
        def query(self,*args):entered.set();release.wait(1)
        def stop(self):pass
    link=Link()
    def state():
        s=d.state(4);s['x5_receive_monotonic_ns']=time.perf_counter_ns();return s
    task=Commissioner(g,link,lock,state,lambda:True)
    with lock:task.start('enable')
    assert entered.wait(.2)
    assert lock.acquire(timeout=.02);lock.release()
    with pytest.raises(ValueError):task.start('enable')
    release.set();task.thread.join(1)
    assert g.commissioning['state']=='complete' and g.enabled_once

def test_async_gripper_fault_is_retained_without_retry():
    import threading
    from e6_observer.control_guardian import Commissioner
    _,d,g=setup('live')
    class Link:
        cancel=threading.Event()
        def grip(self,t):raise ValueError('first feedback stalled')
    def state():
        s=d.state();s['x5_receive_monotonic_ns']=time.perf_counter_ns();return s
    link=Link();task=Commissioner(g,link,threading.RLock(),state,lambda:True,lambda:object())
    task.start('initialize-gripper');task.thread.join(1)
    assert g.commissioning['state']=='failed' and g.gripper_faulted and not g.gripper_initialized
    with pytest.raises(ValueError):task.start('initialize-gripper')

def test_guardian_serves_real_websocket_and_exits_without_shadow_writes():
    import queue,socket,threading
    from types import SimpleNamespace
    from collections import deque
    from websockets.sync.client import connect
    from e6_observer.control_guardian import run_guardian,put_latest
    with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    d=Device(time.perf_counter_ns);d.events=deque();d.gripper=None;d.cancel=threading.Event()
    d.motion_tx=d.stop_tx=0;d.close=lambda:None
    meta=queue.Queue(1);results=queue.Queue(128);heartbeat=SimpleNamespace(value=time.perf_counter_ns())
    shutdown=threading.Event();ready=threading.Event()
    def parent():
        while not shutdown.wait(.02):
            heartbeat.value=time.perf_counter_ns();put_latest(meta,{'rgb':{'x5_receive_monotonic_ns':time.perf_counter_ns()}})
    upkeep=threading.Thread(target=parent,daemon=True);upkeep.start()
    worker=threading.Thread(target=run_guardian,args=('fake','shadow','harness','test-token',{},meta,results,heartbeat,shutdown,ready),kwargs={'link_factory':lambda ip:d,'feedback_source':d.state,'listen_port':port},daemon=True)
    worker.start();assert ready.wait(2)
    try:
        with connect(f'ws://127.0.0.1:{port}/ws/control',additional_headers={'X-E6-Control-Token':'test-token','X-E6-Control-Role':'harness'}) as ws:
            status=json.loads(ws.recv(timeout=1))['status'];time.sleep(.06)
            msg=dict(operation='input',seq=1,lease=status['lease'],source='keyboard',identity='synthetic',connected=True,focused=True,deadman=False,fine=False,stop=False,neutral=True,mode='translation',frame='base',twist=[0]*6,gripper_rate=0,raw={})
            ws.send(json.dumps(msg));assert json.loads(ws.recv(timeout=1))['ok']
            ws.send(json.dumps({'operation':'arm'}));reply=json.loads(ws.recv(timeout=1));assert reply['status']['armed'],reply
            with connect(f'ws://127.0.0.1:{port}/ws/control',additional_headers={'X-E6-Control-Token':'test-token','X-E6-Control-Role':'harness'}) as second:assert not json.loads(second.recv(timeout=1))['ok']
            assert not d.cancel.is_set(),'Rejected second controller must not cancel the owner'
            from websockets.exceptions import ConnectionClosed
            for headers in ({'X-E6-Control-Token':'wrong','X-E6-Control-Role':'harness'},
                            {'X-E6-Control-Token':'test-token','X-E6-Control-Role':'operator'}):
                with connect(f'ws://127.0.0.1:{port}/ws/control',additional_headers=headers) as rejected:
                    with pytest.raises(ConnectionClosed):rejected.recv(timeout=1)
                assert not d.cancel.is_set(),'Unauthenticated connection must not cancel the owner'
            ws.send(json.dumps(dict(msg,seq=2)));assert json.loads(ws.recv(timeout=1))['status']['armed']
            ws.send(json.dumps({'operation':'stop'}));assert json.loads(ws.recv(timeout=1))['ok']
    finally:shutdown.set();worker.join(3);upkeep.join(1)
    assert not worker.is_alive() and d.servo_count==d.grip_count==d.stop_count==0

def test_operator_profile_incomplete_or_missing_critical_sources_rejected(tmp_path):
    from e6_teleop.acceptance_profile import verify_profile
    path=tmp_path/'profile.json';path.write_text(json.dumps({'schema_version':1,'complete':False}))
    with pytest.raises(ValueError,match='incomplete'):verify_profile(path)
    path.write_text(json.dumps(dict(schema_version=1,complete=True,robot_ip='192.168.1.4',camera_serial='AY4A1520111',tool=0,user=0,source_sha256={'irrelevant':'foo'})))
    with pytest.raises(ValueError,match='critical'):verify_profile(path)

def test_shadow_record_does_not_claim_real_action_labels(tmp_path):
    r=ObservationRecorder(tmp_path,'hardware_demonstration');name=r.start('shadow',{'control_stage':'shadow'})
    f=hardware_frame();f['control_stage']='shadow';f['effective_action']=None;r.sample(f);r.finish('success')
    s=json.loads((tmp_path/name/'summary.json').read_text(encoding='utf-8'))
    assert not s['is_real_demonstration'] and not s['action_labels_available'] and s['control_stage']=='shadow'

def test_uncertain_fault_frame_is_retained_separately_and_first_cause_survives(tmp_path):
    recorder=ObservationRecorder(tmp_path,'hardware_demonstration')
    episode=recorder.start('uncertain send',{'control_stage':'live'})
    frame=hardware_frame();frame['status']={'faulted':True}
    frame['images']={'wrist_jpeg':'preview-only'}
    frame['control']={'command_events':[{'command':'ServoJ(...)','uncertain':True,'acknowledged':False}]}
    with pytest.raises(ValueError,match='Control fault'):recorder.sample(frame)
    recorder.retain_failure(dict(frame,control={'command_events':[]}), 'later stop')
    recorder.finish('aborted','unconfirmed command')
    failure=json.loads((tmp_path/episode/'failure_frame.json').read_text(encoding='utf-8'))
    assert not failure['accepted_sample'] and 'images' not in failure['frame']
    assert failure['frame']['control']['command_events'][0]['uncertain']
    assert (tmp_path/episode/'samples.jsonl').read_text()==''
    assert not json.loads((tmp_path/episode/'summary.json').read_text())['complete']

def test_shadow_release_holds_candidate_target_without_reset_to_actual():
    c,d,g=setup();arm(g,d)
    for _ in range(10):send(g,d,deadman=True,neutral=False,twist=[.002,0,0,0,0,0]);tick(c,d,g)
    target=g.target_p.copy();send(g,d)
    for _ in range(3):send(g,d);tick(c,d,g)
    np.testing.assert_array_equal(g.target_p,target)
    assert g.armed and d.servo_count==0

def test_robot_only_zero_servoj_precedes_gripper_initialization():
    c,d,g=setup('live');arm(g,d)
    g.gripper_initialized=False;g.grip_target=None
    send(g,d,deadman=True);r=tick(c,d,g)
    assert d.servo_count==1 and d.grip_count==0 and r['effective_action']['gripper'] is None
    send(g,d,deadman=True,neutral=False,gripper_rate=.05);r=tick(c,d,g)
    assert not g.armed and g.faulted and d.servo_count==1 and d.grip_count==0

def test_rearm_cannot_discard_unconfirmed_stop():
    c,d,g=setup('live');arm(g,d);send(g,d,deadman=True);tick(c,d,g)
    d.stop=lambda:(_ for _ in ()).throw(TimeoutError('Stop reply lost'))
    send(g,d);send(g,d)
    with pytest.raises(ValueError,match='确认停止'):g.command(dict(session=g.owner,operation='arm'),d.state(),c())
    assert not g.armed and g.faulted and g.pending

def test_shadow_ik_seed_is_latest_actual_feedback():
    c,d,g=setup();arm(g,d);seeds=[];old=d.ik
    def ik(p,q,deadline):seeds.append(np.asarray(q));return old(p,q,deadline)
    d.ik=ik
    for _ in range(3):send(g,d,deadman=True,neutral=False,twist=[.002,0,0,0,0,0]);tick(c,d,g)
    for q in seeds:np.testing.assert_allclose(q,np.degrees(d.anchor),rtol=0,atol=1e-10)

def test_control_fault_aborts_raw_and_preserves_prefix(tmp_path):
    r=ObservationRecorder(tmp_path,'hardware_demonstration');name=r.start('fault',{'control_stage':'live'})
    r.sample(hardware_frame());f=hardware_frame(1);f['status']={'faulted':True}
    with pytest.raises(ValueError,match='Control fault'):r.sample(f)
    r.finish('aborted')
    s=json.loads((tmp_path/name/'summary.json').read_text());assert not s['complete'] and s['samples']==1

def test_gripper_feedback_cannot_be_from_after_action(tmp_path):
    r=ObservationRecorder(tmp_path,'hardware_demonstration');r.start('post CAN',{})
    f=hardware_frame();f['sources']['gripper']['x5_receive_monotonic_ns']=f['observation']['control_tick_monotonic_ns']+1
    with pytest.raises(ValueError,match='Gripper observation'):r.sample(f)
    r.finish('aborted')

def test_failed_shadow_never_unlocks_live_harness(tmp_path):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from control_acceptance import require_shadow_acceptance
    with pytest.raises(ValueError,match='passing'):require_shadow_acceptance(None,{})
    path=tmp_path/'shadow.json';path.write_text(json.dumps({'passed':False}))
    with pytest.raises(ValueError,match='failed'):require_shadow_acceptance(path,{'production_input_sha256':'test','rows':[]})

@pytest.mark.parametrize('fault',['length','nan','restart','eof'])
def test_v3_recorded_feedback_split_and_fail_closed(fault):
    from e6_observer.protocol import FeedbackParser
    path=Path(__file__).resolve().parents[2]/'docs/evidence/teleop_observation_20261004/e6_feedback_1440.bin'
    raw=path.read_bytes();p=FeedbackParser(extended=True)
    assert not p.feed(raw[:51]);assert len(p.feed(raw[51:]+raw))==2
    broken=bytearray(raw)
    if fault=='length':struct.pack_into('<H',broken,0,1439)
    elif fault=='nan':struct.pack_into('<d',broken,480,float('inf'))
    elif fault=='restart':struct.pack_into('<Q',broken,32,0)
    if fault=='eof':
        p.feed(raw[:93])
        with pytest.raises(ValueError,match='Truncated'):p.eof()
    else:
        with pytest.raises(ValueError):p.feed(bytes(broken))

@pytest.mark.parametrize('fault',['robot','rgb','disconnect','restart','disk','control'])
def test_v3_pc_fault_aborts_and_preserves_recording(tmp_path,fault):
    import asyncio
    from test_observation import Connection
    class ControlConnection(Connection):
        async def messages(self):
            async for raw in super().messages():
                f=json.loads(raw)
                if 'observation' in f:
                    f.update(schema_version=3,backend='hardware-control',control_stage='shadow',input=None,effective_action=None)
                    f['observation'].update(control_tick_id=f['observation']['frame_id'],control_tick_monotonic_ns=f['observation']['gateway_monotonic_ns'])
                    if fault=='control' and f['observation']['frame_id']>5:f['status']={'faulted':True}
                yield json.dumps(f)
    with patch('e6_teleop.observation_service.websockets.connect',side_effect=lambda *a,**kw:ControlConnection(fault)):
        with TestClient(make_app(tmp_path,render=False,backend='hardware-control',control_gateway='ws://127.0.0.1:2',control_token='test')) as client:
            for _ in range(60):
                if client.get('/api/state').json()['gateway_ready']:break
                time.sleep(.01)
            if fault=='disk':
                with patch.object(ObservationRecorder,'sample',side_effect=OSError('injected write failure')):
                    assert client.post('/api/record/start',json={'task':'v3 fault'}).status_code==200
                    time.sleep(.15)
            else:
                assert client.post('/api/record/start',json={'task':'v3 fault'}).status_code==200
                time.sleep(.4)
            assert not client.get('/api/state').json()['recording']['active']
            episodes=client.get('/api/episodes').json();assert len(episodes)==1
            assert episodes[0]['result']=='aborted' and not episodes[0]['complete']
            assert client.post('/api/export/'+episodes[0]['id']).status_code==400

@pytest.mark.parametrize('fault',['feedback','rgb','parent'])
def test_independent_guardian_watchdog_stops_live_replacement(fault,monkeypatch):
    import queue,socket,threading
    from types import SimpleNamespace
    from collections import deque
    from websockets.sync.client import connect
    from e6_observer import control_guardian as guardian
    class StableCore(ControlCore):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            # Supply already acquired stable history; do not wait five wall seconds
            # for each injected fault. Real commissioning uses unmodified core.
            now=self.clock();q=np.radians([-66,-60,105,-90,90,9]).tolist()
            self.history.extend((now-round((100-i)*50_000_000),q) for i in range(101))
    monkeypatch.setattr(guardian,'ControlCore',StableCore)
    with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    d=Device(time.perf_counter_ns);d.events=deque();d.gripper=None;d.cancel=threading.Event();d.motion_tx=d.stop_tx=0;d.close=lambda:None
    failed=threading.Event();meta=queue.Queue(1);results=queue.Queue(128);heartbeat=SimpleNamespace(value=time.perf_counter_ns())
    shutdown=threading.Event();ready=threading.Event()
    def fresh():
        s=d.state()
        if failed.is_set() and fault=='feedback':s['x5_receive_monotonic_ns']-=200_000_000
        return s
    def parent():
        while not shutdown.wait(.01):
            if not (failed.is_set() and fault=='parent'):heartbeat.value=time.perf_counter_ns()
            ns=time.perf_counter_ns()-(600_000_000 if failed.is_set() and fault=='rgb' else 0)
            guardian.put_latest(meta,{'rgb':{'x5_receive_monotonic_ns':ns}})
    upkeep=threading.Thread(target=parent,daemon=True);upkeep.start()
    worker=threading.Thread(target=guardian.run_guardian,args=('replacement','live','harness','test-token',{},meta,results,heartbeat,shutdown,ready),kwargs={'link_factory':lambda ip:d,'feedback_source':fresh,'listen_port':port},daemon=True)
    worker.start();assert ready.wait(2)
    try:
        with connect(f'ws://127.0.0.1:{port}/ws/control',additional_headers={'X-E6-Control-Token':'test-token','X-E6-Control-Role':'harness'}) as ws:
            status=json.loads(ws.recv(timeout=1))['status'];time.sleep(.06)
            ws.send(json.dumps(dict(operation='input',seq=1,lease=status['lease'],source='keyboard',identity='synthetic',connected=True,focused=True,deadman=False,fine=False,stop=False,neutral=True,mode='translation',frame='base',twist=[0]*6,gripper_rate=0,raw={})))
            assert json.loads(ws.recv(timeout=1))['ok']
            ws.send(json.dumps({'operation':'arm'}));assert json.loads(ws.recv(timeout=1))['status']['armed']
            started=time.perf_counter();failed.set()
            while not d.stop_count and time.perf_counter()-started<.4:time.sleep(.005)
            assert d.stop_count>=1 and d.servo_count==d.grip_count==0
    finally:shutdown.set();worker.join(3);upkeep.join(1)
    assert not worker.is_alive()

def test_servoj_missing_ack_is_uncertain_and_socket_reserved_for_stop():
    import socket,threading
    from collections import deque
    from e6_observer.control_guardian import DashboardLink
    class Socket:
        closed=False
        def settimeout(self,*_):pass
        def sendall(self,*_):pass
        def recv(self,*_):raise socket.timeout()
        def close(self):self.closed=True
    link=DashboardLink.__new__(DashboardLink);link.sock=Socket();sock=link.sock
    link.events=deque();link.motion_tx=link.stop_tx=0;link.io_lock=threading.RLock();link.cancel=threading.Event()
    with pytest.raises(TimeoutError):link.query('ServoJ(0,0,0,0,0,0,t=0.05,aheadtime=50,gain=500)',time.perf_counter_ns()+2_000_000)
    event=link.events[-1]
    assert event['send_confirmed'] and event['uncertain'] and not event['acknowledged']
    assert link.motion_tx==1 and not sock.closed and link.sock is sock
    assert link.late_motion_reply['command']==event['command']
    with pytest.raises(ValueError,match='only independent Stop'):link.query('GetAngle()')
    link.close_socket()
    assert sock.closed and link.sock is None and link.late_motion_reply is None


def test_feedback_failure_keeps_exact_fragmented_bytes_and_fails_closed(tmp_path):
    from e6_observer.feedback_diagnostics import FeedbackTrace
    from e6_observer.protocol import FeedbackParser
    parser=FeedbackParser(extended=True);trace=FeedbackTrace(max_chunks=4)
    good=packet();bad=b'\x00\x00malformed'
    chunks=[good[:37],good[37:]+bad[:1],bad[1:]]
    failure=None
    for i,raw in enumerate(chunks):
        trace.receive(raw,i+1)
        try:
            for native,d in parser.feed(raw):trace.valid(native,d,i+1)
        except ValueError as exc:failure=trace.failure(parser,exc,tmp_path)
    assert failure and failure['prefix_hex']==bad.hex()
    report=json.loads(Path(failure['path']).read_text())
    assert b''.join(base64.b64decode(c['raw_base64']) for c in report['chunks'])==good+bad
    assert base64.b64decode(report['last_valid']['raw_base64'])==good
    assert base64.b64decode(report['parser_buffer_base64'])==bad
    assert report['total_bytes']==len(good+bad)


@pytest.mark.parametrize('stage',['shadow','live'])
def test_release_during_native_ik_does_not_send_or_fault_and_holds_target(stage):
    c,d,g=setup(stage);arm(g,d);before=g.target_p.copy();q=d.q.copy()
    send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    original=d.ik
    def released_ik(*args):
        d.release_pending=True
        return original(*args)
    d.ik=released_ik
    result=tick(c,d,g)
    assert not result['ik_result']['accepted'] and result['effective_action'] is None
    assert g.armed and not g.faulted and d.servo_count==d.grip_count==0
    assert np.array_equal(g.target_p,before) and np.array_equal(d.q,q)
    send(g,d,deadman=False,neutral=True)
    assert g.armed and not g.faulted
    assert d.stop_count==(1 if stage=='live' else 0)


def test_streaming_lease_renews_without_reply_nonce_but_expires_and_rejects_replay():
    c,d,g=setup();lease=g.lease
    for seq in range(6):
        c.step(.1)
        send(g,d,seq=seq,lease=lease)
        assert g.lease==lease
    with pytest.raises(ValueError,match='重复'):
        send(g,d,seq=5,lease=lease)
    assert g.lease!=lease and not g.armed
    fresh=g.lease;c.step(.251)
    with pytest.raises(ValueError,match='租约'):
        send(g,d,seq=6,lease=fresh)
    assert d.servo_count==d.grip_count==0


def test_duplex_bridge_forwards_fresh_input_while_return_path_is_delayed():
    import asyncio
    from e6_teleop.control_transport import StreamingGatewayConnection
    class Wire:
        def __init__(self):self.sent=[];self.responses=asyncio.Queue();self.closed=False
        async def send(self,value):self.sent.append(json.loads(value))
        def __aiter__(self):return self
        async def __anext__(self):return await self.responses.get()
        async def close(self):self.closed=True
    async def exercise():
        wire=Wire();replies=[]
        async def notify(reply):replies.append(reply)
        bridge=StreamingGatewayConnection(wire,{'status':{'session':'owner','lease':'lease'}},notify)
        try:
            # Deliberately no return packets. Neither send waits for an ACK.
            await bridge.send({'operation':'input','seq':1,'request_id':71})
            await bridge.send({'operation':'input','seq':2,'request_id':72})
            assert [v['seq'] for v in wire.sent]==[1,2]
            waiter=asyncio.create_task(bridge.rpc({'operation':'stop'}))
            await asyncio.sleep(.01)
            for command in wire.sent:
                await wire.responses.put(json.dumps(dict(ok=True,status=bridge.status,request_id=command['request_id'])))
            assert (await asyncio.wait_for(waiter,.1))['ok']
            assert [r['request_id'] for r in replies]==[71,72]
        finally:await bridge.close()
        assert wire.closed
    asyncio.run(exercise())


def test_rpc_consumes_future_failure_when_response_reader_already_failed():
    import asyncio
    from e6_teleop.control_transport import StreamingGatewayConnection
    async def exercise():
        errors=[];loop=asyncio.get_running_loop();loop.set_exception_handler(lambda _,context:errors.append(context))
        bridge=StreamingGatewayConnection.__new__(StreamingGatewayConnection)
        async def fail(data,future,notify):
            future.set_exception(ConnectionError('reader disconnected'))
            raise ConnectionError('send disconnected')
        bridge.send=fail
        with pytest.raises(ConnectionError,match='send disconnected'):
            await bridge.rpc({'operation':'stop'})
        await asyncio.sleep(0)
        assert not errors
    asyncio.run(exercise())


def test_normal_release_never_hides_a_native_transaction_timeout():
    c,d,g=setup();arm(g,d)
    send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    original=d.ik
    def late_release(*args):
        d.release_pending=True;c.step(.076)
        return original(*args)
    d.ik=late_release
    result=tick(c,d,g)
    assert g.faulted and not g.armed and not result['ik_result']['accepted']
    assert '75ms' in result['ik_result']['reason']
    assert d.servo_count==d.grip_count==0


def test_internal_feedback_fault_revokes_a_streaming_lease_before_expiry():
    c,d,g=setup();arm(g,d);lease=g.lease
    send(g,d);c.step(.05)
    stale=d.state();stale['x5_receive_monotonic_ns']-=101_000_000
    g.tick(stale,{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert g.faulted and not g.armed and g.lease!=lease
    with pytest.raises(ValueError,match='租约'):
        send(g,d,lease=lease,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    assert d.servo_count==d.grip_count==0


def test_guardian_receive_time_not_lock_acquisition_renews_input():
    c,d,g=setup();arm(g,d)
    received=c();c.step(.12)
    data=dict(g.input,session=g.owner,operation='input',lease=g.lease,seq=g.seq+1,
              gateway_received_ns=c()+10_000_000_000)
    g.command(data,d.state(),c(),received_ns=received)
    assert g.input_ns==g.lease_ns==received
    assert g.input['gateway_received_ns']==received
    assert g.input['gateway_processed_ns']==c()
    c.step(.131)
    g.tick(d.state(),{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert not g.armed and g.faulted and d.servo_count==d.grip_count==0


@pytest.mark.parametrize('delay',[.251,-.001])
def test_stale_or_future_guardian_receive_time_rejected(delay):
    c,d,g=setup();arm(g,d);received=c()-round(delay*1e9)
    data=dict(g.input,session=g.owner,operation='input',lease=g.lease,seq=g.seq+1)
    with pytest.raises(ValueError,match='接收'):
        g.command(data,d.state(),c(),received_ns=received)
    assert not g.armed and d.servo_count==d.grip_count==0


def test_release_stop_failure_revokes_lease_and_never_resumes_old_input():
    c,d,g=setup('live');arm(g,d)
    send(g,d,deadman=True,neutral=False,twist=[.001,0,0,0,0,0]);lease=g.lease
    def failed_stop():raise TimeoutError('missing Stop confirmation')
    d.stop=failed_stop
    send(g,d,deadman=False,neutral=True)
    assert not g.armed and g.faulted and g.stop_acknowledged is False and g.lease!=lease
    with pytest.raises(ValueError,match='租约'):
        send(g,d,lease=lease,deadman=True,neutral=False,twist=[.001,0,0,0,0,0])
    assert d.servo_count==d.grip_count==0


def test_substitute_wireless_results_cannot_unlock_live_hardware(tmp_path):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from control_acceptance import require_shadow_acceptance,source_hashes
    summary=dict(passed=True,stage='shadow',suite='matrix',duration_s=601,invalid=0,actions=0,
        production_input_sha256='fixture',source_sha256=source_hashes(),source_changed_during_run=False,
        cases=[],stats=dict(motion_tx=0,can_tx=0,stop_tx=0),test_fixture=True,hardware_sources_verified=False)
    path=tmp_path/'substitute.json';path.write_text(json.dumps(summary))
    with pytest.raises(ValueError,match='failed'):
        require_shadow_acceptance(path,{'production_input_sha256':'fixture','rows':[]})


def test_temporary_transport_diagnosis_does_not_unlock_live(tmp_path):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from control_acceptance import require_shadow_acceptance,source_hashes
    summary=dict(passed=True,stage='shadow',suite='matrix',duration_s=601,invalid=0,actions=0,
        production_input_sha256='fixture',source_sha256=source_hashes(),source_changed_during_run=False,
        cases=[],stats=dict(motion_tx=0,can_tx=0,stop_tx=0),test_fixture=False,
        hardware_sources_verified=True,diagnostic_transport=True)
    path=tmp_path/'diagnostic.json';path.write_text(json.dumps(summary))
    with pytest.raises(ValueError,match='failed'):
        require_shadow_acceptance(path,{'production_input_sha256':'fixture','rows':[]})
