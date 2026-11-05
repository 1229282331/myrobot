"""One computed point per physical write, including late results and release."""
from copy import deepcopy
import numpy as np
import pytest
from e6_phosphobot.cpp_cycle import CppCycle
from e6_phosphobot.gateway import wait_control_cycle
from test_cpp_cycle import context,response
from test_gateway import setup
from test_moveit_bridge import Bridge,row


def paced_context(now,receipt=None,**kw):
    c=context(now,**kw);c['dispatch']=dict(version=1,receipt=receipt);return c


def start_cycle():
    c=CppCycle(writer_paced=True);now=1_000_000_000
    c.update(paced_context(now,active=False));assert c.request(now)
    c.complete(response(c),now+1_000_000)
    now+=2_000_000;c.update(paced_context(now));assert c.request(now)
    first=c.complete(response(c),now+8_000_000)
    return c,now,first


def receipt(row,stamp):
    return dict(id=row['id'],epoch=row['epoch'],sent_ns=stamp,joints_rad=row['joints_rad'])


def test_solver_never_advances_an_unsent_point_even_when_many_periods_pass():
    c,now,first=start_cycle()
    for ms in (10,50,80,100,200):
        c.update(paced_context(now+ms*1_000_000))
        assert c.request(now+ms*1_000_000) is None
    assert c.previous_candidate['id']==first['id']


def test_written_parent_triggers_one_lookahead_step_and_uses_actual_send_slot():
    c,now,first=start_cycle();sent=now+30_000_000
    c.update(paced_context(sent+5_000_000,receipt(first,sent)))
    assert c.request(sent+5_000_000)
    second=c.complete(response(c),sent+15_000_000)
    assert second['cpp']['parent_id']==first['id']
    assert second['dispatch']['not_before_ns']==sent+50_000_000
    c.update(paced_context(sent+60_000_000,receipt(first,sent)))
    assert c.request(sent+60_000_000) is None


def test_changed_parent_position_is_not_a_valid_commit_receipt():
    c,now,first=start_cycle();r=receipt(first,now+10_000_000);r['joints_rad']=list(r['joints_rad']);r['joints_rad'][0]+=.1
    c.update(paced_context(now+20_000_000,r))
    with pytest.raises(ValueError,match='receipt'):c.request(now+20_000_000)


def test_release_clears_unwritten_point_and_new_epoch_resets_smoother():
    c,now,first=start_cycle();c.update(paced_context(now+20_000_000,active=False))
    assert c.request(now+20_000_000)
    c.complete(response(c),now+21_000_000)
    c.update(paced_context(now+22_000_000,epoch=2))
    assert c.request(now+22_000_000)
    new=c.complete(response(c),now+23_000_000)
    assert new['cpp']['parent_id'] is None and new['cpp']['reset']


class PacedBridge(Bridge):
    def __init__(self):
        super().__init__();self.info['writer_pacing_version']=1;self.epoch=1;self.receipt=None
    def dispatch_snapshot(self):return dict(version=1,receipt=deepcopy(self.receipt))
    def committed(self,r,ns):self.receipt=receipt(r,ns)


def target(now,index,parent,due,sequence):
    r=row(now,sequence)
    r.update(id=index,epoch=1,valid_until_ns=now+100_000_000,
             dispatch=dict(version=1,not_before_ns=due),cpp=dict(parent_id=parent))
    r['joints_rad'][5]=.0001*index
    return r


def test_gateway_holds_ready_endpoint_without_skipping_then_rebases_late_slot():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    start=now[0];send(deadman=True,angular_rad_s=[0,0,.01])
    e.moveit.row=target(start,1,None,start,e.session.sequence);e.tick()
    assert e.moveit.receipt['id']==1
    for index,lag in enumerate((8,20,35,9),2):
        last=e.last_moveit_sent_ns;now[0]=last+lag*1_000_000
        send(deadman=True,angular_rad_s=[0,0,.01])
        e.moveit.row=target(now[0],index,index-1,last+50_000_000,e.session.sequence);e.tick()
        assert e.moveit.receipt['id']==index-1
        assert e.queued_moveit is not None
        # One simulated scheduling miss must not shorten the NEXT period.
        now[0]=last+(68 if index==3 else 50)*1_000_000;e.tick()
        assert e.moveit.receipt['id']==index
        assert e.last_moveit_sent_ns-last>=50_000_000
    assert len([c for c in d.calls if c[0]=='ServoJ'])==5


def test_release_cancels_prepared_but_unwritten_endpoint():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01])
    e.moveit.row=target(now[0],1,None,now[0]+50_000_000,e.session.sequence);e.tick()
    assert e.queued_moveit
    send();now[0]+=50_000_000;e.tick()
    assert e.queued_moveit is None and not any(c[0]=='ServoJ' for c in d.calls)


def test_expiration_during_wait_is_not_extended_by_fresh_feedback():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01])
    e.moveit.row=target(now[0],1,None,now[0]+50_000_000,e.session.sequence);e.tick()
    now[0]+=101_000_000;e.tick()
    assert not any(c[0]=='ServoJ' for c in d.calls)
    assert e.session.ready and e.session.input_paused_ns is not None
    assert e.queued_moveit is None


def test_expiration_after_health_validation_is_still_rejected_before_writer():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01]);start=now[0]
    e.moveit.row=target(start,1,None,start,e.session.sequence)
    original=e.joint_tracking.candidate
    def slow_validation(*args):
        result=original(*args);now[0]+=101_000_000;return result
    e.joint_tracking.candidate=slow_validation;e.tick()
    assert not any(c[0]=='ServoJ' for c in d.calls)
    assert e.session.input_paused_ns is not None


def test_snapshot_publication_between_clock_and_observe_cannot_create_negative_age():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=Bridge();e.moving=True
    old=d.snapshot
    def newer_snapshot():
        result=old();now[0]+=44_083;result['received_ns']=now[0];return result
    d.snapshot=newer_snapshot;e.tick()
    assert not any(x['event']=='joint_tracking_stopped' for x in e.events)


def test_wait_wakes_at_dispatch_slot_without_busy_loop_or_full_period_delay():
    class Event:
        def __init__(self):self.waits=[]
        def wait(self,t):self.waits.append(t);return False
    halt,wake=Event(),Event()
    wait_control_cycle(halt,wake,100_000_000,.05,80_000_000,clock=lambda:120_000_000,
                       producer_paced=True,scheduled_ns=130_000_000)
    assert wake.waits==[.01] and not halt.waits


@pytest.mark.parametrize('reason',['candidate_age','sample_deadline','gripper_path_changed'])
def test_discarded_first_point_cancels_epoch_instead_of_waiting_forever(reason):
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01])
    e.moveit.last_take=dict(reason=reason,id=1,epoch=1)
    e.tick()
    assert not any(c[0]=='ServoJ' for c in d.calls)
    assert e.session.ready and e.session.input_paused_ns is not None


def test_zero_first_point_cancels_epoch_without_acknowledging_unsent_point():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01])
    r=target(now[0],1,None,now[0],e.session.sequence);r['joints_rad']=[0.]*6
    e.moveit.row=r;e.tick()
    assert e.moveit.receipt is None and not any(c[0]=='ServoJ' for c in d.calls)
    assert e.session.ready and e.session.input_paused_ns is not None


@pytest.mark.parametrize('parent',[None,2,99])
def test_unexpected_reset_or_skipped_unsent_parent_is_a_writer_barrier(parent):
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01])
    e.moveit.row=target(now[0],1,None,now[0],e.session.sequence);e.tick()
    now[0]+=50_000_000;send(deadman=True,angular_rad_s=[0,0,.01])
    e.moveit.row=target(now[0],3,parent,now[0],e.session.sequence);e.tick()
    assert len([c for c in d.calls if c[0]=='ServoJ'])==1
    assert e.moveit.receipt['id']==1 and e.session.input_paused_ns is not None
    assert any(x['event']=='motion_dispatch_rejected' and x['reason']=='uncommitted_solver_parent' for x in e.events)
