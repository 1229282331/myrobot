import pytest
from test_gateway import setup


def disabled(stage='live'):
    e,d,send,step,now=setup(stage)
    d.EnableRobot=lambda *a,**kw:d.command('EnableRobot',*a,**kw)
    d.data['status'].update(enable_status=0,robot_mode=4)
    step(7);send()
    return e,d,send,step,now


def test_explicit_enable_does_not_prepare_or_send_motion_and_waits_for_new_stability():
    e,d,send,step,now=disabled()
    assert not d.calls and e.state()['arm']['can_enable']
    send('arm_enable');assert [c[0] for c in d.calls]==['EnableRobot']
    send('arm_enable');assert len(d.calls)==1
    step();assert e.enable_phase=='observing' and not e.session.ready
    send('prepare');assert not e.session.ready
    d.data['status'].update(enable_status=1,robot_mode=5)
    step(90);send();send('prepare');assert not e.session.ready
    step(12);assert e.enable_phase=='complete' and not e.session.ready
    send();send('prepare');assert e.session.ready
    assert [c[0] for c in d.calls]==['EnableRobot']
    assert [event['event'] for event in e.events if event['event'].startswith('enable')]==[
        'enable_requested','enable_result','enable_observed']


def test_disabled_time_does_not_count_towards_enabled_stability():
    e,d,send,step,now=disabled();step(200)
    d.data['status'].update(enable_status=1,robot_mode=5)
    step(2);send();send('prepare')
    assert not e.session.ready and e.state()['arm']['stable_ms']<100
    step(102);send();send('prepare');assert e.session.ready
    assert not d.calls  # External enable is observed, never repeated.


@pytest.mark.parametrize('change',[
    {'enable_status':1,'robot_mode':5},{'error_status':1},{'collision_state':1},
    {'robot_mode':7},{'run_queued_cmd':1},{'tool_coordinate':1}])
def test_ineligible_device_states_never_send_enable(change):
    e,d,send,step,now=disabled();d.data['status'].update(change);send('arm_enable')
    assert not d.calls and e.enable_phase=='idle'


def test_shadow_and_active_input_cannot_enable():
    e,d,send,step,now=disabled('shadow');send('arm_enable');assert not d.calls
    e,d,send,step,now=disabled();send(deadman=True);send('arm_enable');assert not d.calls


def test_enable_reply_failure_preserved_without_retry():
    e,d,send,step,now=disabled();send('arm_enable')
    d.fail=TimeoutError('enable reply lost');step()
    assert e.enable_phase=='uncertain' and e.enable_receipt is not None
    d.fail=None;send();send('arm_enable');step(100)
    assert [c[0] for c in d.calls]==['EnableRobot'] and not e.session.ready


def test_enable_acceptance_does_not_fabricate_enabled_feedback():
    e,d,send,step,now=disabled();send('arm_enable');step(405)
    assert e.enable_phase=='failed' and not e.state()['arm']['enabled']
    assert len(d.calls)==1 and not e.session.ready


def test_gripper_and_prepare_blocked_while_enabling():
    e,d,send,step,now=disabled();send('arm_enable');step()
    d.data['status'].update(enable_status=1,robot_mode=5);step(10)
    send();send('gripper_initialize');assert '稳定观察' in e.session.reason
    send('prepare');assert not e.session.ready


def test_page_loss_collects_enable_result_without_followup_actions():
    e,d,send,step,now=disabled();d.auto_ack=False;send('arm_enable')
    pending=e.enable_pending;e.release('page')
    pending.event.update(accepted=True,acknowledged=True);pending.done.set()
    d.data['status'].update(enable_status=1,robot_mode=5);step(103)
    assert e.enable_phase=='complete' and e.session is None and len(d.calls)==1


def test_stale_feedback_is_not_displayed_as_known_enable_state():
    e,d,send,step,now=disabled();d.data['age_ms']=101;step()
    assert e.state()['arm']['enabled'] is None and not e.state()['arm']['can_enable']


def test_service_exit_collects_one_enable_receipt_without_disabling():
    e,d,send,step,now=disabled();send('arm_enable');e.close()
    assert e.enable_phase=='observing' and e.enable_receipt['accepted']
    assert [c[0] for c in d.calls]==['EnableRobot'] and d.closed


def test_enable_completion_preserves_an_explicit_stop_notice():
    e,d,send,step,now=disabled();send('arm_enable');step()
    d.data['status'].update(enable_status=1,robot_mode=5)
    send('stop');step(103)
    assert e.enable_phase=='complete' and not e.session.ready
    assert e.session.reason=='operator_stop'


@pytest.mark.parametrize('failure',['generation','stale','alarm','camera'])
def test_enabled_observation_interruption_never_prepares_or_retries(failure):
    e,d,send,step,now=disabled();send('arm_enable');step()
    d.data['status'].update(enable_status=1,robot_mode=5);step(20)
    if failure=='generation':d.data['generation']='replacement-feedback'
    elif failure=='stale':d.data['age_ms']=101
    elif failure=='alarm':d.data['status']['error_status']=1
    else:e.camera=lambda:dict(valid=False)
    step()
    assert e.enable_phase=='failed' and not e.session.ready
    assert len(d.calls)==1
