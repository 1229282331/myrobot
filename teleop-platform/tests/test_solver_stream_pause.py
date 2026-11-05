"""Original deadline loss must stop without silently resuming held input."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from test_cpp_cycle import pending, response, context
from test_gateway import setup
from test_moveit_bridge import Bridge, row, intent


def starved():
    e, d, send, step, now = setup()
    e.moveit = Bridge()
    aborted = []
    e.episodes = SimpleNamespace(state=lambda: {'phase': 'capturing'}, abort=aborted.append)
    send(deadman=True, linear_m_s=[.002, 0, 0])
    e.moveit.row = row(now[0]); step()
    old_echo = e.session.clock_tick()
    # Mirror the observed three missing polls, without extending 150 ms.
    for _ in range(3):
        send(deadman=True, linear_m_s=[.002, 0, 0]); step()
    assert d.calls[-1][0] == 'ServoJ'
    now[0] += 1
    e.tick()
    return e, d, send, step, now, aborted, old_echo


def test_stream_gap_stops_once_keeps_recording_and_accepts_fresh_reverse():
    e, d, send, step, now, aborted, old_echo = starved()
    assert e.session.ready and e.state()['input_paused']
    assert e.state()['direction_limited'] and e.state()['state']=='可控制'
    assert '控制保持' in e.state()['reason']
    assert [c[0] for c in d.calls] == ['ServoJ', 'Stop']
    assert not aborted and e.moveit.row is None and e.target is None
    events = [x for x in e.events if x['event'] == 'model_stream_paused']
    assert len(events) == 1 and events[0]['candidate_gap_ms'] > 150
    for i in range(20):
        send(deadman=True, linear_m_s=[.001 if i%2 else .002, 0, 0]); step()
    assert [c[0] for c in d.calls] == ['ServoJ', 'Stop']
    e.receive(dict(op='intent', seq=e.session.sequence+1, echo_ns=old_echo,
                   source='keyboard', client_sample_ms=0,
                   intent=intent(linear_m_s=[0]*3, deadman=False).model_dump()))
    assert e.session.input_paused_ns is not None
    send(deadman=True, linear_m_s=[-.002, 0, 0])
    assert e.session.input_paused_ns is None and e.state()['reason'] is None
    assert not e.state()['direction_limited']
    e.moveit.row = row(now[0], e.session.sequence); step()
    assert d.calls[-1][0] == 'ServoJ' and len(d.calls) == 3 and not aborted
    assert len([x for x in e.events if x['event']=='control_direction_limited'])==1


def test_reverse_waits_for_confirmed_stationary_stop_and_recomputed_candidate():
    e,d,send,step,now,aborted,_=starved()
    receipt=e.stop_pending
    receipt.done.clear()
    now[0]+=1
    send(deadman=True,linear_m_s=[-.002,0,0])
    assert e.session.input_paused_ns is None
    for _ in range(8):
        send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert [c[0] for c in d.calls]==['ServoJ','Stop']
    assert e.stop_pending is receipt and not e.stop_acknowledged
    receipt.done.set()
    d.data['status']['robot_mode']=7
    for _ in range(8):
        send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert e.stop_acknowledged and not e.stationary and len(d.calls)==2
    d.data['status']['robot_mode']=5
    for _ in range(8):
        send(deadman=True,linear_m_s=[-.002,0,0]);step()
    assert e.stationary and e.session.ready and len(d.calls)==2
    e.moveit.row=row(now[0],e.session.sequence);step()
    assert d.calls[-1][0]=='ServoJ' and len(d.calls)==3 and not aborted


def test_directional_pause_can_recover_to_gripper_only_without_arm_write():
    e,d,send,step,now,aborted,_=starved()
    rates=[]
    e.gripper=SimpleNamespace(snapshot=lambda:dict(phase='ready',actual=.5),
        hold=lambda:None,set_stroke_rate=lambda rate,dt:rates.append(rate))
    now[0]+=1
    for _ in range(12):
        send(deadman=True,gripper_rate_s=.02);step()
    assert rates and all(x==.02 for x in rates)
    assert [c[0] for c in d.calls]==['ServoJ','Stop']
    assert e.session.ready and not e.state()['input_paused'] and not aborted


def test_invalid_unsent_target_keeps_prepare_but_never_reaches_device():
    e,d,send,step,now=setup();e.moveit=Bridge()
    aborted=[]
    e.episodes=SimpleNamespace(state=lambda:dict(phase='capturing'),abort=aborted.append)
    send(deadman=True,linear_m_s=[.002,0,0])
    e.moveit.row=row(now[0])
    e.moveit.row['joints_rad'][4]=.01
    step()
    assert not d.calls
    assert e.session.ready and e.session.reason=='solver_target_discontinuity'
    assert e.joint_tracking.last_detail['reason']=='joint_command_discontinuity'
    for _ in range(8):
        send(deadman=True,linear_m_s=[.002,0,0]);step()
    assert not d.calls and not aborted
    send(deadman=True,linear_m_s=[-.002,0,0])
    e.moveit.row=row(now[0],e.session.sequence);step()
    assert [c[0] for c in d.calls]==['ServoJ'] and not aborted


@pytest.mark.parametrize('fault', ['feedback', 'model', 'stop_rejected', 'stop_uncertain', 'operator_stop', 'disconnect'])
def test_stream_pause_does_not_hide_hard_faults_or_explicit_stop(fault):
    e, d, send, step, now, aborted, _ = starved()
    if fault == 'feedback':
        d.data['connected'] = False; step()
    elif fault == 'model':
        e.moveit.info.update(ready=False, error='worker failed'); step()
    elif fault == 'stop_rejected':
        e.stop_pending.event['accepted'] = False; step()
    elif fault == 'stop_uncertain':
        d.fail=TimeoutError('stop receipt uncertain'); step()
    elif fault == 'operator_stop':
        send('stop')
    else:
        e.release('page')
    assert not e.session or not e.session.ready
    assert [c[0] for c in d.calls] == ['ServoJ', 'Stop']
    assert aborted
    # A software pause is not allowed to reclassify the later hard failure.
    assert e.reason != 'solver_stream_stale'


def test_late_solver_diagnostic_survives_neutral_and_later_success():
    c, now = pending(); reply = response(c)
    reply['status'] = 5
    reply['callback_us'] = 61000
    reply['gripper_check'] = dict(arm_reason='callback_deadline')
    assert c.complete(reply, now+63_000_000) is None
    detail = deepcopy(c.info['last_rejection_detail'])
    assert detail['reason'] == 'solver_deadline' and detail['status'] == 5
    assert detail['id'] == reply['seq'] and detail['epoch'] == reply['epoch']
    assert detail['roundtrip_ms'] == 63 and detail['callback_us'] == 61000
    assert detail['deadline_ns'] == now+100_000_000
    assert detail['started_ns'] == now and detail['at_ns'] == now+63_000_000
    assert detail['gripper_check']['arm_reason'] == 'callback_deadline'
    c.update(context(now+65_000_000)); assert c.request(now+65_000_000)
    assert c.complete(response(c), now+66_000_000) is None
    assert c.info['status'] == 5 and c.info['last_rejection_detail'] == detail
    c.update(context(now+67_000_000,epoch=2,active=False))
    assert c.request(now+115_000_000)
    assert c.complete(response(c),now+116_000_000) is None
    c.update(context(now+117_000_000,epoch=3))
    assert c.request(now+117_000_000)
    assert c.complete(response(c),now+118_000_000)
    assert c.snapshot(now+118_000_000)['info']['last_rejection_detail'] == detail
    # Later mutation of the response cannot rewrite the recorded failure.
    reply['gripper_check']['arm_reason'] = 'changed'
    assert c.info['last_rejection_detail'] == detail
