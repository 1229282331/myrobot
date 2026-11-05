"""Recorded v293 gripper states must not turn transport activity into motion."""
from copy import deepcopy
import threading
import pytest
from e6_phosphobot.gripper_geometry import request_state,retreat_hold
from test_gripper_contact import contact,attached
NOW=1_000_000_000

def held():
    return dict(valid=True,phase='ready',actual=204/255,received_ns=NOW,sent_raw=204,
                command_raw=204,inflight_raw=None,reported_state=0,reported_velocity_raw=0,
                arrived=True,position_status='held',contact=None)

@pytest.mark.parametrize('inflight',[None,204])
@pytest.mark.parametrize('raw',[203,204,205])
def test_recorded_same_target_polling_is_stationary_but_commitments_are_retained(raw,inflight):
    s=held();s.update(actual=raw/255,inflight_raw=inflight)
    assert retreat_hold(s,NOW)==1
    assert request_state(s,NOW)==[raw,204,-1 if inflight is None else 204,204]

def test_recorded_contact_mismatch_is_frozen_not_an_unfinished_command():
    s=held();s.update(actual=131/255,sent_raw=123,command_raw=123,reported_state=2,
        arrived=False,position_status='contact',contact=dict(direction=-1,escape_requested=False,escape_pending=False))
    assert retreat_hold(s,NOW)==2
    assert request_state(s,NOW)==[131,123,-1,123]
    for change in ({'direction':1},{'escape_requested':True},{'escape_pending':True}):
        other=deepcopy(s);other['contact'].update(change);assert retreat_hold(other,NOW)==0

@pytest.mark.parametrize('change',[
    {'valid':False},{'phase':'fault'},{'received_ns':NOW-200_000_001},{'received_ns':NOW+1},
    {'inflight_raw':205},{'command_raw':205},{'reported_state':1},{'reported_state':3},
    {'reported_velocity_raw':1},{'reported_velocity_raw':None},{'reported_velocity_raw':False},
    {'arrived':False},{'actual':200/255}])
def test_uncertainty_and_new_motion_cannot_inherit_a_hold(change):
    s=held();s.update(change);assert retreat_hold(s,NOW)==0

def test_native_snapshot_carries_velocity_and_contact_escape_revokes_hold_immediately():
    g,d,t=contact();s=g.snapshot()
    assert s['reported_velocity_raw']==0 and retreat_hold(s,t[0])==2
    g.set_rate(.05,.05)
    assert retreat_hold(g.snapshot(),t[0])==0

def test_passive_connection_needs_stable_arrival_and_does_not_send_a_target():
    g,d,t=attached();s=g.snapshot()
    assert retreat_hold(s,t[0])==1 and s['sent_raw'] is None and not d.sent


def test_old_binary_cannot_be_paired_with_new_hold_protocol():
    from e6_phosphobot.jazzy_cpp_worker import verify_dynamic_protocol
    for manifest in ({},{'dynamic_gripper_version':2},{'dynamic_gripper_version':3},
                     {'dynamic_gripper_version':3,'gripper_hold_protocol':'G2'}):
        with pytest.raises(ValueError,match='G3'):verify_dynamic_protocol(manifest)
    verify_dynamic_protocol({'dynamic_gripper_version':3,'gripper_hold_protocol':'G3'})


def test_real_gripper_snapshot_keeps_hold_during_same_target_exchange():
    g,d,t=attached();g.set_target(204/255)
    for _ in range(4):g.cycle();t[0]+=50_000_000
    assert retreat_hold(g.snapshot(),t[0])==1
    entered=threading.Event();released=threading.Event();original=d.exchange
    def exchange(target,**kw):
        entered.set();assert released.wait(2);return original(target,**kw)
    d.exchange=exchange;worker=threading.Thread(target=g.cycle);worker.start()
    assert entered.wait(1)
    try:
        s=g.snapshot()
        assert s['inflight_raw']==s['sent_raw']==s['command_raw']==204
        assert retreat_hold(s,t[0])==1
        from e6_phosphobot.cpp_cycle import CppCycle
        from test_cpp_cycle import context
        cycle=CppCycle(dynamic_gripper=True);ctx=context(t[0],active=False);ctx['gripper']=s;cycle.update(ctx)
        assert cycle.request(t[0]).split()[-6:]==['G3','204','204','204','204','1']
    finally:released.set();worker.join(2)
    assert not worker.is_alive()
