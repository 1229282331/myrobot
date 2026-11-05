from copy import deepcopy
from types import SimpleNamespace
import json
import numpy as np
import pytest
from e6_phosphobot.gateway import NativeExecutor
from e6_phosphobot.dashboard import Pending


def sample():
    return dict(connected=True,age_ms=1,received_ns=1,generation='a',status=dict(
        enable_status=1,error_status=0,collision_state=0,robot_mode=5,run_queued_cmd=0,
        load=.750,load_center_x=0.,load_center_y=-14.,load_center_z=46.,
        user_coordinate=0,tool_coordinate=0,user_value=[0]*6,tool_value=[0]*6,
        joint_state=dict(q_actual=[0]*6,qd_actual=[0]*6,q_target=[0]*6),
        tool_vector_actual=dict(x=0,y=0,z=400,rx=0,ry=0,rz=0)))


class Device:
    def __init__(self):
        self.data=sample();self.calls=[];self.uncertain=False;self._connected=True
        self.closed=False;self.fail=None;self.auto_ack=True;self.reads=0
    def snapshot(self):
        self.reads+=1
        return deepcopy(self.data)
    def start(self):pass
    def close(self):self.closed=True
    def disconnect(self):self.closed=True
    def command(self,name,*args,**kwargs):
        p=Pending(name,dict(command=name,accepted=self.auto_ack,acknowledged=self.auto_ack))
        if self.auto_ack:p.done.set()
        self.calls.append((name,args,kwargs,p));return p
    def Stop(self):return self.command('Stop')
    def ServoP(self,*a,**kw):return self.command('ServoP',*a,**kw)
    def ServoJ(self,*a,**kw):return self.command('ServoJ',*a,**kw)
    def poll(self,p):
        if self.fail:raise self.fail
        return p.done.is_set()


def setup(stage='live',**options):
    now=[1_000_000_000]
    e=NativeExecutor('test',stage,None,clock=lambda:now[0],**options)
    d=Device();e.reader=e.dashboard=e.robot=e.motion=d
    # The fake is a continuously observed device. Keep wire-receive time on
    # the same simulated clock as the new independent writer freshness guard.
    original_snapshot=d.snapshot
    def fresh_snapshot():
        d.data['received_ns']=now[0]-1_000_000
        return original_snapshot()
    d.snapshot=fresh_snapshot
    e.camera=lambda:dict(valid=True)
    e.claim('page')
    def step(count=1):
        for _ in range(count):
            now[0]+=50_000_000;e.tick()
    step(105)
    def send(op='intent',**intent):
        data=dict(op=op,seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
                  source='keyboard',client_sample_ms=0)
        if op=='intent':data['intent']=dict(linear_m_s=[0]*3,angular_rad_s=[0]*3,
            joint_rad_s=[0]*6,gripper_rate_s=0,deadman=False,frame='base')|intent
        e.receive(data)
    send();send('prepare')
    assert e.session.ready
    return e,d,send,step,now


def test_live_stream_uses_native_servo_without_waiting_for_idle():
    e,d,send,step,_=setup()
    for _ in range(10):
        send(deadman=True,linear_m_s=[.002,0,0]);step()
        d.data['status'].update(robot_mode=7,run_queued_cmd=1)
    assert len(d.calls)==10
    assert all(c[0]=='ServoP' for c in d.calls)
    np.testing.assert_allclose(d.calls[-1][1][0][:3],[1,0,400])
    send();step()
    assert d.calls[-1][0]=='Stop' and e.session.ready


def test_model_startup_requires_a_new_explicit_prepare_after_ready():
    e,d,send,_,_=setup()
    e.session.ready=False
    model=dict(ready=False,error='starting')
    resets=[]
    e.moveit=SimpleNamespace(snapshot=lambda:dict(model),stop=lambda:resets.append(True))
    send('prepare')
    assert not e.session.ready and '运动模型尚未就绪' in e.session.reason
    assert not d.calls and not resets
    model.update(ready=True,error=None)
    assert not e.session.ready and not d.calls
    send('prepare')
    assert e.session.ready and resets==[True] and not d.calls

@pytest.mark.parametrize('stage',['shadow','live'])
@pytest.mark.parametrize('ending',['stop','disconnect','fault'])
def test_unprepared_observation_recording_survives_neutral_inputs_but_not_real_stop(stage,ending):
    e,d,send,step,now=setup(stage)
    e.session.ready=False
    reasons=[];phase=['idle']
    def request(op,**data):
        assert op=='start'
        phase[0]='capturing'
    def abort(reason):
        if phase[0]=='capturing':reasons.append(reason);phase[0]='aborted'
    e.episodes=SimpleNamespace(state=lambda:dict(phase=phase[0]),request=request,abort=abort)
    e.receive(dict(op='record_start',seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
                   source='keyboard',client_sample_ms=0,task='stationary observation'))
    assert phase[0]=='capturing'
    for _ in range(20):
        message=dict(op='intent',seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
                     source='keyboard',client_sample_ms=0,intent=dict(linear_m_s=[0]*3,
                     angular_rad_s=[0]*3,joint_rad_s=[0]*6,gripper_rate_s=0,deadman=False,frame='base'))
        e.notify_received('page',message)
        step()  # Consume the socket's cancellation before serialized receive.
        e.receive(message)
    assert phase[0]=='capturing' and not reasons and not e.session.ready
    assert not d.calls
    if ending=='stop':send('stop')
    elif ending=='disconnect':e.release('page')
    else:e.suspend('test device fault')
    assert phase[0]=='aborted' and len(reasons)==1


@pytest.mark.parametrize('source',['keyboard','gamepad'])
def test_research_context_passes_formal_message_validation_and_gateway_without_device_write(source):
    e,d,send,step,now=setup(stage='shadow')
    captured=[]
    e.episodes=SimpleNamespace(request=lambda op,**data:captured.append((op,data)),state=lambda:{'phase':'idle'})
    # Fresh centered intent, followed by the exact envelope used by the page.
    e.session.source=source
    data=dict(op='record_start',seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),
              source=source,client_sample_ms=0,task='test',research_context={'scene':'桌面','operator':'auto'})
    e.receive(data)
    assert len(captured)==1 and captured[0][0]=='start'
    assert captured[0][1]['research_context']==data['research_context']
    assert d.calls==[]
    data.update(seq=data['seq']+1,research_context={'scene':'x'*501})
    e.receive(data)
    assert len(captured)==1 and d.calls==[]


def test_native_gain_qualification_is_explicit_and_reported_for_both_modes():
    e,d,send,step,_=setup()
    assert e.servo_gain==500
    e.servo_gain=750
    e.servo_aheadtime=20
    send(deadman=True,linear_m_s=[0,0,.002]);step()
    assert d.calls[-1][2]['gain']==750
    assert d.calls[-1][2]['aheadtime']==20
    send();step()
    send(deadman=True,joint_rad_s=[0,0,0,0,0,.01]);step()
    assert d.calls[-1][0]=='ServoJ' and d.calls[-1][2]['gain']==750
    assert e.state()['capabilities']['servo_gain']==750
    assert e.state()['capabilities']['servo_aheadtime']==20
    with pytest.raises(ValueError,match='gain'):
        NativeExecutor('test','shadow',None,servo_gain=1001)


def test_zero_axes_activation_is_not_a_motion_command():
    e,d,send,step,_=setup()
    for _ in range(10):send(deadman=True);step()
    send();step()
    assert not d.calls and e.session.ready


def test_native_idle_input_wakes_tick_without_sending_or_integrating_idle_time():
    e,d,send,step,now=setup()
    now[0]+=1_000_000
    send(deadman=True,linear_m_s=[.002,0,0])
    assert e.native_wake.is_set() and not d.calls
    e.native_wake.clear();e.tick()
    np.testing.assert_allclose(d.calls[-1][1][0][:3],[.1,0,400])
    assert e.native_started and e.last_native_sent_ns==now[0]
    send(deadman=True,linear_m_s=[.002,0,0])
    assert not e.native_wake.is_set()  # Stream updates keep the normal cadence.
    send();assert not e.native_started and not e.native_wake.is_set()
    step();send(deadman=True,linear_m_s=[-.002,0,0])
    assert e.native_wake.is_set()


def test_native_centered_axes_stop_with_deadman_held_and_preserve_session():
    e,d,send,step,_=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    send(deadman=True);step()
    assert d.calls[-1][0]=='Stop' and e.session.ready and not e.native_started
    step();send(deadman=True,linear_m_s=[-.002,0,0])
    assert e.native_wake.is_set()


def test_native_wakeup_still_checks_actual_feedback_before_sending():
    e,d,send,step,_=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);assert e.native_wake.is_set()
    d.data['age_ms']=101;step()
    assert not d.calls and not e.session.ready and not e.native_wake.is_set()


def test_shadow_can_prepare_from_disabled_actual_feedback_without_device_writes():
    e,d,send,step,_=setup('shadow')
    e.session.suspend('new disabled observation');e.request_stop()
    d.data['status'].update(enable_status=0,robot_mode=4)
    step(110);send();send('prepare')
    assert e.session.ready and e.stationary
    for _ in range(4):send(deadman=True,linear_m_s=[.001,0,0]);step()
    assert e.candidate is not None and not d.calls
    send();assert not d.calls


def test_live_still_rejects_disabled_feedback_and_never_enables_implicitly():
    e,d,send,step,_=setup()
    d.data['status'].update(enable_status=0,robot_mode=4)
    step();send();send('prepare')
    assert not e.session.ready and e.healthy(d.data)=='机械臂未使能'
    assert not d.calls


def test_episode_finish_stops_once_and_only_sealing_blocks_prepare():
    e,d,send,step,_=setup()
    class Journal:
        phase='capturing'
        def state(self):return dict(phase=self.phase)
        def request(self,op,**data):
            assert op=='finish' and data['result']=='success'
            self.phase='sealing'
        def abort(self,reason):
            if self.phase=='capturing':self.phase='aborted'
    e.episodes=Journal()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    e.receive(dict(op='record_finish',result='success',seq=e.session.sequence+1,echo_ns=0,source='keyboard',client_sample_ms=0))
    assert not e.session.ready and d.calls[-1][0]=='Stop'
    assert e.episodes.phase=='sealing'
    step(110);send();send('prepare')
    assert not e.session.ready
    e.episodes.phase='transfer_pending';send('prepare')
    assert e.session.ready


def test_action_event_contains_original_pre_send_observation():
    e,d,send,step,_=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    event=next(r for r in e.drain_events() if r['event']=='motion_submitted')
    original=event['pre_action_observation']['status']['joint_state']['q_actual']
    assert event['candidate']['kind']=='cartesian'
    assert event['candidate']['joints_rad'] is None  # Controller IK targets are not observed joints.
    assert event['candidate']['observed_joints_rad']==[0]*6
    assert event['candidate']['source_input']['sequence']==event['candidate']['input_sequence']
    assert event['candidate']['source_input']['intent']['linear_m_s']==[.002,0,0]
    d.data['status']['joint_state']['q_actual'][0]=999
    assert original==[0]*6


def test_shadow_recording_requires_fresh_rgb_even_without_control():
    e,_,_,_,_=setup('shadow')
    e.camera=lambda:dict(valid=False)
    assert e.healthy(e.reader.snapshot()) is None
    assert e.recording_health()=='录制的RGB源已过期'


def test_slow_ack_drops_cycles_without_queue_or_catchup_and_release_stops_now():
    e,d,send,step,_=setup();d.auto_ack=False
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    for _ in range(3):send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert len(d.calls)==1
    send()
    assert [c[0] for c in d.calls]==['ServoP','Stop']
    reads=d.reads;step(10)
    assert d.reads>=reads+10 and not e.stop_acknowledged


def test_one_hundred_start_release_cycles_need_no_service_restart():
    e,d,send,step,_=setup()
    for _ in range(100):
        send(deadman=True,linear_m_s=[.002,0,0]);step()
        send();step()
        assert e.session.ready and e.stop_acknowledged
    assert len([c for c in d.calls if c[0]=='ServoP'])==100
    assert len([c for c in d.calls if c[0]=='Stop'])==100


def test_shadow_has_no_dashboard_or_device_writes():
    e,d,send,step,_=setup('shadow');e.dashboard=e.robot=e.motion=None
    send(deadman=True,linear_m_s=[.002,0,0]);step();send();step()
    assert e.candidate and not e.accepted and not d.calls
    assert json.loads(json.dumps(e.state(True)))['stationary'] is True


def test_expired_stop_keeps_observation_and_explicit_recovery_possible():
    e,d,send,step,_=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step();send()
    d.fail=TimeoutError('stop lost');step()
    assert e.stop_failed and not e.session.ready and e.stop_pending is None
    d.fail=None;d.uncertain=True
    def reconnect():d.uncertain=False
    e._connect_dashboard=reconnect
    step(110);send();send('prepare')
    assert e.session.ready and not e.stop_failed
    assert len([x for x in d.calls if x[0]=='Stop'])==1


def test_boundary_is_normal_and_can_retreat():
    e,d,send,step,_=setup('shadow')
    e.target=(np.array([.05,0,.4]),e.target[1],e.target[2])
    d.data['status']['tool_vector_actual']['x']=50
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert e.session.ready and '边界' in e.reason
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert e.candidate['position_m'][0]<.05 and e.reason is None


@pytest.mark.parametrize('fault',['feedback','camera','collision','generation'])
def test_faults_stop_while_preserving_feedback(fault):
    e,d,send,step,_=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    if fault=='feedback':d.data['age_ms']=101
    if fault=='camera':e.camera=lambda:dict(valid=False)
    if fault=='collision':d.data['status']['collision_state']=1
    if fault=='generation':d.data['generation']='b'
    step()
    assert not e.session.ready and any(c[0]=='Stop' for c in d.calls)
    reads=d.reads;step();assert d.reads>reads


def test_failed_dashboard_start_releases_raw_reader():
    e=NativeExecutor('test','live',None);e.reader=Device()
    def fail():raise ConnectionError('offline')
    e._connect_dashboard=fail
    with pytest.raises(ConnectionError):e.start()
    assert e.reader.closed


def test_stop_failure_never_skips_cleanup():
    e,d,send,step,_=setup();e.moving=True
    def fail():raise RuntimeError('disconnect failure')
    e.dashboard=SimpleNamespace(disconnect=fail)
    with pytest.raises(Exception):e.close()
    assert d.closed


@pytest.mark.parametrize('joint',[False,True])
def test_official_30ms_period_matches_integrator_and_command_without_catchup(joint):
    e,d,send,step,now=setup(servo_period_s=.03)
    intent={'joint_rad_s':[.01,0,0,0,0,0]} if joint else {'linear_m_s':[.002,0,0]}
    for gap in (30_000_000,90_000_000):
        send(deadman=True,**intent)
        now[0]+=gap;e.tick()
    assert len(d.calls)==2
    assert all(c[2]['t']==.03 for c in d.calls)
    if joint:assert d.calls[-1][1][0][0]==pytest.approx(np.rad2deg(.0006))
    else:assert d.calls[-1][1][0][0]==pytest.approx(.12)
    assert e.state()['capabilities']['servo_period_s']==.03


def test_unknown_servo_period_rejected_before_device_connections():
    with pytest.raises(ValueError):NativeExecutor('test','live',None,servo_period_s=.001)


@pytest.mark.parametrize('joint',[False,True])
def test_sdk_duration_does_not_change_integration_or_release(joint):
    trajectories=[]
    for duration in (.05,.1):
        e,d,send,step,now=setup(servo_duration_s=duration)
        intent={'joint_rad_s':[.01,0,0,0,0,0]} if joint else {'linear_m_s':[.002,0,0]}
        for gap in (50_000_000,90_000_000):
            send(deadman=True,**intent);now[0]+=gap;e.tick()
        assert len(d.calls)==2 and all(c[2]['t']==duration for c in d.calls)
        trajectories.append([c[1][0] for c in d.calls])
        # An outstanding ACK does not postpone the independent Stop request
        # to the SDK duration. No future target is generated on release.
        d.auto_ack=False
        send(deadman=True,**intent);step()
        before=now[0];send()
        assert now[0]==before and [c[0] for c in d.calls][-1]=='Stop'
        assert len(d.calls)==4
        assert e.state()['capabilities']['servo_period_s']==.05
        assert e.state()['capabilities']['servo_duration_s']==duration
    np.testing.assert_allclose(trajectories[0],trajectories[1])


@pytest.mark.parametrize('value',[True,'0.1',float('nan'),float('inf'),0,-.1,.04,1])
def test_unknown_duration_rejected_before_feedback_creation(monkeypatch,value):
    def forbidden(*a,**kw):raise AssertionError('Feedback must not be created')
    monkeypatch.setattr('e6_phosphobot.gateway.FeedbackReader',forbidden)
    with pytest.raises(ValueError,match='duration'):
        NativeExecutor('test','live',None,servo_duration_s=value)


def test_official_aheadtime_is_explicit_and_does_not_change_rate():
    e,d,send,step,_=setup()
    e.servo_aheadtime=20
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert d.calls[0][2]==dict(t=.05,aheadtime=20,gain=500)
    assert d.calls[0][1][0][0]==pytest.approx(.1)
    with pytest.raises(ValueError):NativeExecutor('test','live',None,servo_aheadtime=0)


def test_gripper_fault_does_not_terminate_arm_observation_or_stop_healthy_arm():
    from e6_phosphobot.gripper import Gripper
    e,d,send,step,_=setup()
    e.gripper=Gripper();e.gripper.fault('stalled')
    reads=d.reads;step(10)
    assert d.reads>=reads+10 and e.session.ready and not d.calls
    assert e.state()['actual']['gripper'] is None
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert d.calls[-1][0]=='ServoP'


def test_uninitialized_gripper_input_is_not_an_arm_motion():
    from e6_phosphobot.gripper import Gripper
    e,d,send,step,_=setup();e.gripper=Gripper()
    send(deadman=True);step()  # Holding Space/LB before touching the gripper axis.
    send(deadman=True,gripper_rate_s=.02);step();send();step()
    assert not d.calls and e.session.ready


def test_gripper_release_and_expiry_discard_pending_batch_without_arm_motion():
    from e6_phosphobot.gripper import Gripper
    e,d,send,step,now=setup()
    class Can:
        def __init__(self):self.targets=[]
        def exchange(self,target,*,velocity=None):
            self.targets.append(target);raw=round(target*255)
            return dict(position_percent=raw*100/255,position_raw=raw,state=0,host_perf_counter_ns=now[0])
    can=Can();g=Gripper(device_factory=lambda _:can,clock=lambda:now[0]);e.gripper=g
    g.initialize()
    for _ in range(3):g.cycle()
    for _ in range(20):send(deadman=True,gripper_rate_s=-.025);step()
    # Let release overtake a CAN cycle: none of the unsent batch may survive.
    assert g.snapshot()['command_raw']<89
    send();g.cycle()
    assert round(can.targets[-1]*255)==89 and e.session.ready and not d.calls
    for _ in range(20):send(deadman=True,gripper_rate_s=-.025);step()
    now[0]+=251_000_000;e.tick();g.cycle()
    assert round(can.targets[-1]*255)==89 and e.session.ready and e.session.input_paused_ns is not None and not d.calls


@pytest.mark.parametrize('source',['feedback','rgb','disabled','collision'])
@pytest.mark.parametrize('already_suspended',[False,True])
def test_external_health_holds_initialized_gripper_without_latching_can_fault(source,already_suspended):
    from e6_phosphobot.gripper import Gripper
    e,d,send,step,now=setup()
    class Can:
        def __init__(self):self.targets=[]
        def exchange(self,target,*,velocity=None):
            self.targets.append(target);raw=round(target*255)
            return dict(position_percent=raw*100/255,position_raw=raw,state=0,host_perf_counter_ns=now[0])
    can=Can();g=Gripper(device_factory=lambda _:can,clock=lambda:now[0]);e.gripper=g
    g.initialize()
    for _ in range(3):g.cycle()
    send(deadman=True,gripper_rate_s=-.025)
    g.set_target(80/255)  # Unsent batch must not survive external health loss.
    if already_suspended:e.suspend('录制结束；回传完成后请重新准备')
    if source=='feedback':d.data['age_ms']=101
    elif source=='rgb':e.camera=lambda:dict(valid=False)
    elif source=='disabled':d.data['status']['enable_status']=0
    else:d.data['status']['collision_state']=1
    step();g.cycle()
    assert not e.session.ready and g.snapshot()['phase']=='ready'
    assert round(can.targets[-1]*255)==89 and not d.calls
    assert not any(r['event']=='gripper_fault' for r in g.drain())
    d.data=sample();e.camera=lambda:dict(valid=True)
    for _ in range(105):send();step();g.cycle()
    assert not e.session.ready  # Fresh telemetry does not resume old input.
    send('prepare')
    assert e.session.ready
    for _ in range(20):send(deadman=True,gripper_rate_s=-.025);step();g.cycle()
    assert round(can.targets[-1]*255)<89 and not d.calls


def test_external_health_loss_during_gripper_initialization_remains_fault():
    from e6_phosphobot.gripper import Gripper
    e,d,send,step,now=setup();e.gripper=Gripper(clock=lambda:now[0])
    e.gripper.initialize();d.data['age_ms']=101;step()
    assert e.gripper.snapshot()['phase']=='fault'

def test_auto_gripper_attachment_waits_for_health_runs_once_and_does_not_prepare():
    from e6_phosphobot.gripper import Gripper
    e,d,send,step,now=setup();e.suspend('test');g=Gripper(clock=lambda:now[0]);e.gripper=g
    e.camera=lambda:dict(valid=False);step();assert g.phase=='uninitialized'
    e.camera=lambda:dict(valid=True);step();assert g.phase=='connecting'
    assert not e.session.ready and not d.calls
    g.fault('stalled')
    for _ in range(10):step()
    assert g.phase=='fault' and not e.session.ready and not d.calls
    assert len([ev for ev in g.drain() if ev['event']=='gripper_connecting'])==1


def test_observer_state_agrees_with_controller_without_issuing_an_input_tick():
    e,d,send,step,_=setup()
    ticks=set(e.session.issued_ticks)
    assert e.state()['ready'] is True
    assert e.session.issued_ticks==ticks
    e.suspend('test')
    assert e.state()['reason']=='test' and not e.state()['ready']


def test_transient_input_loss_stops_preserves_preparation_and_requires_new_release():
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert d.calls[-1][0]=='ServoP'
    now[0]+=300_000_000;step()
    assert d.calls[-1][0]=='Stop' and e.session.ready
    assert e.state()['input_paused'] and e.state()['state']=='暂挂'
    before=sum(c[0]=='ServoP' for c in d.calls)
    for _ in range(10):send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert sum(c[0]=='ServoP' for c in d.calls)==before
    send();step();assert not e.state()['input_paused']
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert sum(c[0]=='ServoP' for c in d.calls)>before and e.session.ready
    e.release('page');assert not e.session or not e.session.ready
