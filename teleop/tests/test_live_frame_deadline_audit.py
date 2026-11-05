import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_live_frames import audit_transaction


def failed_cycle():
    return {'control_tick_monotonic_ns':0,'ik_result':{'accepted':False}},dict(
        command_id=13284,transaction_started_ns=100, gateway_sent_ns=10_000_000,
        gateway_accepted_ns=50_136_240,stop_requested_after_acceptance=True,cycle_completed=False)


def test_late_known_ack_is_preserved_only_as_a_stopped_failed_cycle():
    tick,action=failed_cycle()
    missed=audit_transaction(tick,action,False)
    assert missed['command_id']==13284 and missed['transaction_ms']==50.13614
    with pytest.raises(AssertionError):audit_transaction(tick,action,True)


@pytest.mark.parametrize('missing',['stop_requested_after_acceptance','cycle_completed','rejected-ik'])
def test_audit_cannot_silently_excuse_late_acceptance(missing):
    tick,action=failed_cycle()
    if missing=='rejected-ik':tick['ik_result']['accepted']=True
    else:action.pop(missing)
    with pytest.raises(AssertionError):audit_transaction(tick,action,False)


def test_explicit_75ms_policy_accepts_soft_jitter_without_rewriting_legacy_policy():
    tick,action=failed_cycle()
    action.update(stop_requested_after_acceptance=False,cycle_completed=True)
    tick.update(status={'limits':{'transaction_s':.075}},ik_result={'accepted':True})
    assert audit_transaction(tick,action,True) is None
    tick.pop('status')
    with pytest.raises(AssertionError):audit_transaction(tick,action,True)


def test_75ms_hard_deadline_still_requires_a_retained_stopped_failure():
    tick,action=failed_cycle();tick['status']={'limits':{'transaction_s':.075}}
    action['gateway_accepted_ns']=76_000_100
    assert audit_transaction(tick,action,False)['transaction_ms']==76
    with pytest.raises(AssertionError):audit_transaction(tick,action,True)
