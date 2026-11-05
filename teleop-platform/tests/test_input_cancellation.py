"""Socket-arrival versus locked computation, using production executor paths."""
import threading
import pytest
from test_gateway import setup
from test_moveit_bridge import Bridge,row


def release(e,**changes):
    return dict(op='intent',seq=e.session.sequence+1,echo_ns=e.session.clock_tick(),source='keyboard',client_sample_ms=0,
        intent=dict(linear_m_s=[0]*3,angular_rad_s=[0]*3,joint_rad_s=[0]*6,
                    gripper_rate_s=0,deadman=False,frame='base'))|changes


@pytest.mark.parametrize('outside_workspace',[False,True])
def test_release_arrival_cancels_locked_candidate_before_send_or_boundary(outside_workspace):
    e,d,send,step,now=setup();e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0],e.session.sequence);step()
    message=release(e);entered=threading.Event();resume=threading.Event()
    candidate=row(now[0],e.session.sequence)
    if outside_workspace:candidate['position_m']=[1,0,.4]
    def slow_take():
        entered.set();assert resume.wait(2);return candidate
    e.moveit.take=slow_take
    worker=threading.Thread(target=step);worker.start()
    try:
        assert entered.wait(2)
        previous=(e.session.sequence,e.session.received_ns,e.session.ready)
        e.notify_received('page',message)
        assert (e.session.sequence,e.session.received_ns,e.session.ready)==previous
    finally:resume.set();worker.join(2)
    assert not worker.is_alive()
    assert [c[0] for c in d.calls]==['ServoJ','Stop']
    assert not any(event['event']=='workspace_blocked' for event in e.events)
    e.receive(message);step()
    assert e.session.ready and e.pending_input_cancellation is None and e.stop_acknowledged
    e.moveit.take=lambda:row(now[0],e.session.sequence)
    send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop','ServoJ']


@pytest.mark.parametrize('action',['release','stop'])
def test_arrival_marker_stops_once_without_renewing_or_latching_session(action):
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    message=release(e)
    if action=='stop':message.update(op=action,intent=None)
    previous=e.session.received_ns
    e.notify_received('page',message);step(3)
    assert [c[0] for c in d.calls]==['ServoP','Stop'] and e.session.received_ns==previous
    e.receive(message);assert e.pending_input_cancellation is None
    if action=='release':
        assert e.session.ready
        send(deadman=True,linear_m_s=[.002,0,0]);step()
        assert d.calls[-1][0]=='ServoP'


def test_out_of_order_second_owner_and_invalid_packet_cannot_cancel_or_clear_marker():
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    message=release(e)
    e.notify_received('other',message)
    e.notify_received('page',dict(message,seq=e.session.sequence))
    e.notify_received('page',dict(message,unexpected=True))
    assert e.pending_input_cancellation is None
    e.notify_received('page',message)
    marker=e.pending_input_cancellation
    e.receive(dict(message,seq=e.session.sequence))
    assert e.pending_input_cancellation==marker
    e.receive(message)
    assert e.pending_input_cancellation is None and e.session.ready


def test_stale_echo_release_is_cancelled_but_not_renewed_and_reconnect_clears_owner():
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    message=release(e,echo_ns=0);previous=e.session.received_ns
    e.notify_received('page',message);step();e.receive(message)
    assert [c[0] for c in d.calls]==['ServoP','Stop']
    assert e.session.received_ns==previous and e.session.ready
    e.notify_disconnected('other');assert e.pending_input_cancellation is None
    e.notify_disconnected('page')
    e.release('other');assert e.pending_input_cancellation==('page',None)
    e.release('page');assert e.pending_input_cancellation is None
    assert e.claim('next') and not e.session.ready


@pytest.mark.parametrize('moveit',[False,True])
@pytest.mark.parametrize('outside_workspace',[False,True])
def test_workspace_computation_release_discards_target(monkeypatch,moveit,outside_workspace):
    from e6_phosphobot import gateway
    e,d,send,step,now=setup()
    if moveit:e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    if moveit:
        e.moveit.row=row(now[0],e.session.sequence);step()
        e.moveit.row=row(now[0],e.session.sequence)
    calls=[c[0] for c in d.calls]
    original=gateway.evaluate_workspace
    def evaluation(*args):
        e.notify_received('page',release(e))
        result=original(*args)
        if outside_workspace:result.update(allowed=False,reason='injected boundary')
        return result
    monkeypatch.setattr(gateway,'evaluate_workspace',evaluation)
    step()
    assert [c[0] for c in d.calls]==calls+['Stop']
    assert not any(event['event']=='workspace_blocked' for event in e.events)


def test_cancelled_new_target_does_not_discard_pending_original_receipt():
    e,d,send,step,now=setup();d.auto_ack=False
    send(deadman=True,linear_m_s=[.002,0,0]);step();pending=e.pending
    message=release(e);e.notify_received('page',message);step()
    assert e.pending is pending and e.stop_pending is not None
    assert [c[0] for c in d.calls]==['ServoP','Stop']
    for _,_,_,item in d.calls:
        item.event.update(accepted=True,acknowledged=True);item.done.set()
    e.receive(message);step()
    assert e.stop_acknowledged and e.pending is None and e.accepted['receipt']['accepted']


def test_shadow_release_cancels_computation_without_device_commands():
    e,d,send,step,now=setup('shadow');e.moveit=Bridge()
    send(deadman=True,linear_m_s=[.002,0,0]);e.moveit.row=row(now[0],e.session.sequence)
    message=release(e);e.notify_received('page',message);step();e.receive(message)
    assert not d.calls and e.moveit.row is None and e.session.ready


def test_experimental_ruckig_release_keeps_its_explicit_semantics():
    e,d,send,step,now=setup(release_mode='ruckig')
    e.notify_received('page',release(e))
    assert e.pending_input_cancellation is None
    e.notify_received('page',release(e,op='stop',intent=None))
    assert e.pending_input_cancellation is not None


@pytest.mark.parametrize('action',['neutral','record_finish_unconfigured','record_finish_missing_result'])
def test_rejected_non_release_does_not_stop_then_resume_old_input(action):
    from types import SimpleNamespace
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    message=release(e,echo_ns=0)
    if action=='neutral':message['intent']['deadman']=True
    else:
        message.update(op='record_finish',intent=None)
        if action=='record_finish_unconfigured':message['result']='failure'
        else:e.episodes=SimpleNamespace(request=lambda *a,**kw:pytest.fail('Invalid finish was accepted'))
    original=e.session.intent
    e.notify_received('page',message)
    assert e.pending_input_cancellation is None
    e.receive(message);step()
    assert e.session.intent is original and e.session.ready
    assert [c[0] for c in d.calls]==['ServoP','ServoP']


@pytest.mark.parametrize('notification',['newer_release','disconnect'])
def test_receiving_prior_release_cannot_erase_newer_cancellation(monkeypatch,notification):
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    message=release(e);e.notify_received('page',message)
    original=e._receive
    def receive_with_notification(data):
        original(data)
        if notification=='newer_release':e.notify_received('page',release(e))
        else:e.notify_disconnected('page')
    monkeypatch.setattr(e,'_receive',receive_with_notification)
    e.receive(message)
    expected=None if notification=='disconnect' else message['seq']+1
    assert e.pending_input_cancellation==('page',expected)
    step(2)
    assert [c[0] for c in d.calls]==['ServoP','Stop']


def test_disconnect_cannot_be_overwritten_by_delayed_release_or_an_old_owner():
    e,d,send,step,now=setup()
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    message=release(e)
    e.notify_disconnected('page');e.notify_received('page',message)
    assert e.pending_input_cancellation==('page',None)
    e.receive(message);step(2)
    assert [c[0] for c in d.calls]==['ServoP','Stop']
    assert e.pending_input_cancellation==('page',None)
    e.release('page');assert e.claim('next')
    e.notify_disconnected('page');e.notify_received('page',message)
    assert e.pending_input_cancellation is None and not e.session.ready


@pytest.mark.parametrize('axis_nonzero',[False,True])
def test_arrival_release_freezes_gripper_target_before_serialized_processing(axis_nonzero):
    from types import SimpleNamespace
    e,d,send,step,now=setup();held=[]
    send(deadman=True,linear_m_s=[.002,0,0]);step()
    e.gripper=SimpleNamespace(hold=lambda:held.append(True),snapshot=lambda:dict(phase='unknown'))
    message=release(e)
    if axis_nonzero:message['intent']['linear_m_s']=[.002,0,0]
    e.notify_received('page',message);step()
    assert held and [c[0] for c in d.calls]==['ServoP','Stop']
    assert e.session.intent.deadman  # The serialized handler has not run yet.


def test_newer_marker_is_not_replaced_by_out_of_order_arrival():
    e,d,send,step,now=setup()
    message=release(e)
    e.notify_received('page',dict(message,seq=message['seq']+1))
    e.notify_received('page',message)
    assert e.pending_input_cancellation==('page',message['seq']+1)


def test_release_during_gripper_snapshot_does_not_commit_new_opening():
    from types import SimpleNamespace
    e,d,send,step,now=setup();held=[];targets=[]
    send(deadman=True,gripper_rate_s=.02)
    def grip_snapshot():
        e.notify_received('page',release(e))
        return dict(phase='ready',limits=dict(minimum=.15,maximum=.6),target=.35)
    # First snapshot is the health check; inject during the later calculation.
    snapshots=[dict(phase='ready')]
    e.gripper=SimpleNamespace(hold=lambda:held.append(True),set_target=targets.append,
        snapshot=lambda:snapshots.pop() if snapshots else grip_snapshot())
    step()
    assert held and not targets and not d.calls
