"""Servo endpoint and independent single-socket Stop confirmation regressions."""
from collections import deque
import copy
import socket
import threading
import time
import numpy as np
import pytest
from test_hardware_control import setup,arm,send,tick
from e6_observer.control_guardian import DashboardLink


@pytest.mark.parametrize('violation',[None,'old-receive','early','old-id','future-id','target','actual','cartesian','rotation','wrong-mode','late-processing'])
def test_streaming_endpoint_requires_fresh_exact_id_bounded_tracking_and_duration(violation):
    c,d,g=setup('live');arm(g,d);send(g,d,deadman=True);tick(c,d,g)
    pending=copy.deepcopy(g.pending);c.step(.05);send(g,d,deadman=True)
    state=d.state(7);state['decoded']['run_queued_command']=1
    if violation=='old-receive':state['x5_receive_monotonic_ns']=pending['accepted_ns']
    if violation=='early':state['x5_receive_monotonic_ns']=pending['accepted_ns']+49_000_000
    if violation=='old-id':state['decoded']['current_command_id']-=1
    if violation=='future-id':state['decoded']['current_command_id']+=1
    if violation=='target':state['decoded']['q_target_deg'][0]+=.051
    if violation=='actual':state['decoded']['joints_rad'][0]+=float(np.radians(.051))
    if violation=='cartesian':state['decoded']['controller_pose_raw_mm_deg'][0]+=1.01
    if violation=='rotation':state['decoded']['controller_pose_raw_mm_deg'][3]+=1.01
    if violation=='wrong-mode':state['decoded']['robot_mode']=10
    if violation=='late-processing':c.step(.11)
    result=g.tick(state,{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert d.servo_count==(2 if violation is None else 1)
    completed=[e for e in result['events'] if e['kind']=='servoj_endpoint_completed']
    assert bool(completed)==(violation is None)
    if completed:assert completed[0]['command_id']==pending['command_id']


class AsyncDevice:
    def __init__(self,device):self.device=device;self.result=None;self.requests=0
    def __getattr__(self,key):return getattr(self.device,key)
    def begin_stop(self):
        self.requests+=1;self.result=dict(confirmation_complete=False,acknowledged=False);return dict(self.result)
    def poll_stop(self):return self.result and dict(self.result)


def test_explicit_stop_before_first_target_confirms_once_without_motion():
    c,d,g=setup('live');a=AsyncDevice(d);g.device=a
    assert g.stop_acknowledged is None and not g.armed
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    original=g.stop_requested_ns
    assert a.requests==1 and g.stop_acknowledged is False
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    assert a.requests==1 and g.stop_requested_ns==original
    a.result.update(confirmation_complete=True,acknowledged=True)
    g.poll_stop()
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    assert a.requests==1 and g.stop_acknowledged is True
    assert d.servo_count==d.grip_count==0


def test_shadow_explicit_stop_before_first_target_never_writes():
    c,d,g=setup('shadow')
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    assert d.stop_count==d.servo_count==d.grip_count==0


def test_accepted_action_exposes_exact_transaction_start_for_budget_audit():
    c,d,g=setup('live');arm(g,d);send(g,d,deadman=True)
    result=tick(c,d,g);action=result['effective_action']
    assert result['control_tick_monotonic_ns']<=action['transaction_started_ns']<=action['gateway_sent_ns']
    assert action['gateway_accepted_ns']-action['transaction_started_ns']<=50_000_000


def test_pending_stop_blocks_new_targets_rearm_and_automatic_retransmission():
    c,d,g=setup('live');arm(g,d);a=AsyncDevice(d);g.device=a
    send(g,d,deadman=True);tick(c,d,g);send(g,d)
    assert g.armed and g.pending and a.requests==1
    for _ in range(6):send(g,d);tick(c,d,g)
    assert not g.faulted and d.servo_count==1 and a.requests==1
    with pytest.raises(ValueError,match='确认停止'):
        g.command(dict(operation='arm',session=g.owner),d.state(),c())
    for _ in range(5):g.stop('watchdog',force=True,fault=True)
    assert a.requests==1
    a.result.update(confirmation_complete=True,acknowledged=True)
    tick(c,d,g)
    assert g.stop_acknowledged and not g.pending
    assert d.servo_count==1


def test_missing_stop_confirmation_faults_and_never_reuses_old_input():
    c,d,g=setup('live');arm(g,d);a=AsyncDevice(d);g.device=a
    send(g,d,deadman=True);tick(c,d,g);send(g,d);lease=g.lease
    a.result.update(confirmation_complete=True,error='missing Stop reply')
    tick(c,d,g)
    assert not g.armed and g.faulted and g.lease!=lease and a.requests==1
    for _ in range(5):tick(c,d,g)
    assert a.requests==1 and d.servo_count==1


@pytest.mark.parametrize('violation',[None,'braking-target','queued-stop','small-tracking-lag','tracks-braking-target','excess-braking-lag','no-send','complete','expired','wrong-id','target','actual','queue','mode','error','collision'])
def test_owned_stop_queue_pause_transition_never_permits_motion_or_rearm(violation):
    c,d,g=setup('live');arm(g,d);a=AsyncDevice(d);g.device=a
    send(g,d,deadman=True);tick(c,d,g);send(g,d)
    a.result.update(send_confirmed=True);g.poll_stop();c.step()
    s=d.state(7);s['decoded']['pause_command_flag']=1
    if violation=='no-send':a.result['send_confirmed']=False;g.poll_stop()
    if violation=='complete':g.stop_result['confirmation_complete']=True
    if violation=='expired':c.step(1.001);s['x5_receive_monotonic_ns']=c()
    if violation=='wrong-id':s['decoded']['current_command_id']+=1
    if violation=='braking-target':s['decoded']['q_target_deg'][0]+=.084454
    if violation=='queued-stop':s['decoded']['run_queued_command']=1
    if violation=='target':s['decoded']['q_target_deg'][0]+=.10001
    if violation=='small-tracking-lag':s['decoded']['joints_rad'][5]+=float(np.radians(.05138439157655661))
    if violation=='actual':s['decoded']['joints_rad'][0]+=float(np.radians(.10001))
    if violation in ('tracks-braking-target','excess-braking-lag'):
        s['decoded']['q_target_deg'][5]+=.09271191582217497
        s['decoded']['joints_rad'][5]+=float(np.radians(.1056704424335031 if violation=='tracks-braking-target' else .14273))
    if violation=='queue':s['decoded']['run_queued_command']=2
    if violation=='mode':s['decoded']['robot_mode']=10
    if violation=='error':s['decoded']['error_status']=1
    if violation=='collision':s['decoded']['collision_state']=1
    if violation in (None,'braking-target','queued-stop','small-tracking-lag','tracks-braking-target'):
        g.safety(s,c(),c())
        g.tick(s,{'rgb':{'x5_receive_monotonic_ns':c()}})
        assert not g.faulted and not g.stopped_feedback and d.servo_count==1 and a.requests==1
        with pytest.raises(ValueError,match='确认停止'):
            g.command(dict(operation='arm',session=g.owner),d.state(),c())
        # Pressing again cannot use the stopping transition to send a target.
        send(g,d,deadman=True);g.tick(s,{'rgb':{'x5_receive_monotonic_ns':c()}})
        assert d.servo_count==1 and a.requests==1
    else:
        with pytest.raises(ValueError):g.safety(s,c(),c())


def test_explicit_stop_during_pending_confirmation_preserves_original_request():
    c,d,g=setup('live');arm(g,d);a=AsyncDevice(d);g.device=a
    send(g,d,deadman=True);tick(c,d,g);send(g,d)
    requested=g.stop_requested_ns
    c.step(.2)
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    assert a.requests==1 and g.stop_requested_ns==requested
    assert not g.stop_acknowledged and not g.stopped_feedback
    a.result.update(confirmation_complete=True,error='missing Stop reply')
    tick(c,d,g)
    g.command(dict(operation='stop',session=g.owner),d.state(),c())
    assert a.requests==2 and g.stop_requested_ns>requested


def test_stationary_feedback_requires_new_receives_after_stop():
    c,d,g=setup('live');arm(g,d);g.stop('explicit stop')
    old=d.state();old['x5_receive_monotonic_ns']-=50_000_000
    for _ in range(3):
        c.step();g.tick(old,{'rgb':{'x5_receive_monotonic_ns':c()}})
    assert not g.stopped_feedback
    for _ in range(3):tick(c,d,g)
    assert g.stopped_feedback


def link_with_socket(wire):
    link=DashboardLink.__new__(DashboardLink);link.sock=wire;link.events=deque()
    link.motion_tx=link.stop_tx=0;link.io_lock=threading.RLock();link.cancel=threading.Event()
    link.stop_state=None;link.stop_thread=None;link.stop_state_lock=threading.Lock();link.gripper=None
    return link


@pytest.mark.parametrize('reply,ack',[(b'0,{},Stop();',True),(b'0,{8},ServoJ();',False),(b'-1,{},Stop();',False),(b'',False)])
def test_single_stop_receives_only_its_delayed_reply_without_blocking_or_retransmitting(reply,ack):
    class Wire:
        def __init__(self):self.sent=[];self.gate=threading.Event();self.closed=False
        def settimeout(self,*_):pass
        def sendall(self,b):self.sent.append(b)
        def recv(self,*_):
            if not self.gate.wait(.01):raise socket.timeout()
            return reply
        def close(self):self.closed=True
    wire=Wire();link=link_with_socket(wire)
    try:
        start=time.perf_counter();result=link.begin_stop()
        assert time.perf_counter()-start<.05 and result['send_confirmed']
        assert not result['confirmation_complete']
        with pytest.raises(ValueError,match='pending'):link.query('GetAngle()')
        link.begin_stop();assert wire.sent==[b'Stop()'] and link.stop_tx==1
        time.sleep(.06);wire.gate.set();link.stop_thread.join(.5)
        result=link.poll_stop();assert result['confirmation_complete'] and result['acknowledged']==ack
        assert wire.sent==[b'Stop()']
        if ack:assert result['confirmation_duration_ms']>=50 and not wire.closed
        else:assert wire.closed and result['error']
    finally:wire.gate.set();link.close()


def test_stop_confirmation_deadline_discards_late_socket_without_resend():
    class Wire:
        sent=[];closed=False
        def settimeout(self,*_):pass
        def sendall(self,b):self.sent.append(b)
        def recv(self,*_):time.sleep(.01);raise socket.timeout()
        def close(self):self.closed=True
    wire=Wire();link=link_with_socket(wire)
    try:
        link.begin_stop();link.stop_thread.join(1.5)
        result=link.poll_stop()
        assert result['confirmation_complete'] and not result['acknowledged']
        assert '1000ms' in result['error'] and wire.closed and wire.sent==[b'Stop()']
    finally:link.close()


@pytest.mark.parametrize('reply_mode',['split','coalesced','wrong-motion','duplicate-motion','rejected-motion','missing-stop','extra-stop'])
def test_timed_out_motion_retains_one_connection_for_correlated_stop(reply_mode):
    command='ServoJ(0,0,0,0,0,0,t=0.05,aheadtime=50,gain=500)'
    class Wire:
        def __init__(self):self.sent=[];self.closed=False;self.parts=deque();self.stop_sent=False
        def settimeout(self,*_):pass
        def setsockopt(self,*_):pass
        def sendall(self,b):
            self.sent.append(b)
            if b==b'Stop()':
                self.stop_sent=True
                late=('0,{9},'+command+';').encode()
                if reply_mode=='wrong-motion':late=b'0,{9},ServoJ(1,1,1,1,1,1);'
                if reply_mode=='rejected-motion':late=('-1,{},'+command+';').encode()
                stop=b'0,{},Stop();'
                if reply_mode=='duplicate-motion':stop=late+stop
                if reply_mode=='missing-stop':stop=b''
                if reply_mode=='extra-stop':stop+=b'0,{},Stop();'
                payload=late+stop
                self.parts.extend([payload[:7],payload[7:33],payload[33:]] if reply_mode=='split' else [payload])
        def recv(self,*_):
            if self.parts:return self.parts.popleft()
            time.sleep(.001);raise socket.timeout()
        def close(self):self.closed=True
    wire=Wire();link=link_with_socket(wire)
    try:
        with pytest.raises(TimeoutError):link.query(command,time.perf_counter_ns()+10_000_000)
        motion=link.events[0]
        assert motion['uncertain'] and not motion['acknowledged'] and not wire.closed
        with pytest.raises(ValueError,match='only independent Stop'):link.query('GetAngle()')
        link.begin_stop();link.stop_thread.join(1.4)
        result=link.poll_stop()
        expected=reply_mode in ('split','coalesced','rejected-motion')
        assert result['confirmation_complete'] and result['acknowledged']==expected
        assert wire.sent==[command.encode(),b'Stop()']
        assert not motion['acknowledged'] and motion['uncertain']
        if expected:
            assert motion['late_reply_ignored_for_action'] and not wire.closed
        else:assert wire.closed
    finally:link.close()


def test_delivery_caps_only_available_to_startup_fixed_automatic_harness():
    from e6_teleop.tunnel import Tunnel
    configured=Tunnel('127.0.0.1',control_stage='shadow',control_role='harness',harness_limits='delivery')
    assert configured.harness_limits=='delivery'
    for kwargs in ({},{'control_stage':'live','control_role':'operator'},
                   {'control_stage':'live','control_role':'harness','acceptance_profile':{'complete':False}}):
        with pytest.raises(ValueError,match='harness-only'):Tunnel('127.0.0.1',harness_limits='delivery',**kwargs)
