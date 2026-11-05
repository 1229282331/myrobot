"""Delivery slack covers the measured stall without extending live input."""
from copy import deepcopy

import pytest

from e6_phosphobot.cpp_cycle import CppCycle, fresh_actual
from e6_phosphobot.delivery_timing import (
    PROFILE, candidate_deadline_ns, configure_delivery, stream_timeout_ns,
    validate_delivery,
)
from e6_phosphobot.moveit_bridge import MoveItBridge
from test_cpp_cycle import context, response
from test_gateway import setup
from test_writer_pacing import PacedBridge, target


def cycle_ready():
    c=CppCycle(writer_paced=True,callback_budget_ms=55,request_timeout_ms=70,
               delivery_timing=PROFILE)
    now=1_000_000_000
    ctx=context(now,active=False)
    c.update(ctx);assert c.request(now)
    c.complete(response(c),now+1_000_000)
    now+=2_000_000
    ctx=context(now);ctx['dispatch']=dict(version=1,receipt=None)
    c.update(ctx);assert c.request(now)
    return c,now


def extended(row,source=None):
    row=deepcopy(row)
    source=row['received_ns'] if source is None else source
    row['cpp']['delivery_timing']=dict(PROFILE)
    row['pre_action_received_ns']=source
    row['source_input']=dict(received_ns=source)
    row['valid_until_ns']=source+150_000_000
    return row


def test_manifest_is_opt_in_and_cannot_extend_arbitrary_routes():
    assert configure_delivery({},True) is None
    manifest=dict(delivery_timing=PROFILE,solver_timing={'version':1})
    assert configure_delivery(manifest,True)==PROFILE
    for m,paced in ((manifest,False),(dict(delivery_timing=PROFILE),True)):
        with pytest.raises(ValueError):configure_delivery(m,paced)
    for change in ({'source_lifetime_ms':151},{'stream_timeout_ms':300},{'version':True}):
        with pytest.raises(ValueError):validate_delivery(PROFILE|change)
    assert stream_timeout_ns({})==150_000_000
    assert stream_timeout_ns(dict(writer_pacing_version=1,delivery_timing=PROFILE))==200_000_000


def test_slow_source_then_short_computation_keeps_original_bounded_lease():
    c,now=cycle_ready()
    c.pending['actual']['received_ns']-=40_000_000
    c.update(context(now+65_000_000)|{'dispatch':dict(version=1,receipt=None)})
    row=c.complete(response(c),now+65_000_000)
    assert row and row['valid_until_ns']==now+110_000_000
    assert row['cpp']['period_s']==.05
    assert candidate_deadline_ns(row)==now+110_000_000
    # No step beyond the outstanding unsent point, including after expiry.
    for ms in (70,109,111,170):
        ctx=context(now+ms*1_000_000);ctx['dispatch']=dict(version=1,receipt=None)
        c.update(ctx);assert c.request(now+ms*1_000_000) is None


def test_live_feedback_request_timeout_and_input_deadline_are_not_extended():
    c,now=cycle_ready()
    assert not fresh_actual(context(now)['actual'],now+101_000_000)
    assert c.complete(response(c),now+70_000_000) is None
    c,now=cycle_ready()
    c.pending['command']['received_ns']-=200_000_000
    c.update(context(now+60_000_000)|{'dispatch':dict(version=1,receipt=None)})
    assert c.complete(response(c),now+60_000_000) is None
    assert c.info['last_rejection']=='solver_deadline'


def test_context_gap_cannot_reset_smoother_behind_a_pending_writer_receipt():
    from test_writer_pacing import receipt
    c,now=cycle_ready();first=c.complete(response(c),now+10_000_000)
    assert not c.reset_required
    # Pause only forwarding to the worker. Its existing input is still live;
    # the gateway's own feedback stream is checked separately at dispatch.
    assert c.request(now+110_000_000) is None
    observed=c.snapshot(now+110_000_000)
    assert observed['candidate'] is first and observed['info']['context_waiting']
    assert observed['info']['ready'] and not c.reset_required
    assert c.previous_candidate['id']==first['id']
    ctx=context(now+125_000_000)
    ctx['dispatch']=dict(version=1,receipt=receipt(first,now+120_000_000))
    c.update(ctx);assert c.request(now+125_000_000)
    following=c.complete(response(c),now+130_000_000)
    assert following['cpp']['parent_id']==first['id'] and not following['cpp']['reset']
    assert following['dispatch']['not_before_ns']==now+170_000_000
    assert not c.snapshot(now+130_000_000)['info']['context_waiting']


def test_pending_reply_can_use_delivery_slack_without_an_implicit_reset():
    c,now=cycle_ready()
    c.pending['actual']['received_ns']-=40_000_000
    c.context['actual']['received_ns']-=40_000_000
    row=c.complete(response(c),now+65_000_000)
    assert row and row['valid_until_ns']==now+110_000_000
    assert c.snapshot(now+65_000_000)['info']['context_waiting']
    assert not c.reset_required


def test_expired_unseen_point_is_delivered_for_rejection_not_hidden_forever():
    c,now=cycle_ready();row=c.complete(response(c),now+10_000_000)
    ctx=context(now+151_000_000);ctx['dispatch']=dict(version=1,receipt=None);c.update(ctx)
    expired=c.snapshot(now+151_000_000)['candidate']
    assert expired is row and expired['valid_until_ns']==now+150_000_000
    b=MoveItBridge('.',None,clock=lambda:now+151_000_000,api='cpp')
    b.epoch=row['epoch'];b.command=ctx['command'];b.latest=expired
    assert b.take() is None and b.last_take['reason']=='candidate_age'
    c.update(context(now+152_000_000,active=False))
    assert c.snapshot(now+152_000_000)['candidate'] is None


def test_expired_paced_point_wakes_for_cancellation_without_announcing_usable_start():
    from test_candidate_inbox import setup as inbox_setup, packet
    b,pipe,inbox,now=inbox_setup('cpp')
    row=packet(b,1,now[0]-20_000_000,now[0]-1)
    row['candidate']['dispatch']=dict(version=1)
    pipe.row=row;inbox.receive()
    assert b.initial_candidate_event.is_set()
    assert b.first_candidate is None
    assert b.take() is None and b.last_take['reason']=='sample_deadline'


@pytest.mark.parametrize('field',['pre_action_received_ns','source_input'])
def test_extended_candidate_requires_original_sources(field):
    row=extended(target(1_000_000_000,1,None,1_000_000_000,1))
    del row[field]
    with pytest.raises(ValueError):candidate_deadline_ns(row)


def test_deadline_cannot_be_rebased_to_new_publication_or_fresh_feedback():
    row=extended(target(1_000_000_000,1,None,1_000_000_000,1))
    row.update(received_ns=1_100_000_000,valid_until_ns=1_250_000_000)
    assert candidate_deadline_ns(row)==1_150_000_000
    row['source_input']['received_ns']=800_000_000
    assert candidate_deadline_ns(row)==1_050_000_000


@pytest.mark.parametrize('delay_ms,accepted',[(101,True),(140,True),(150,False),(175,False)])
def test_final_writer_uses_same_original_delivery_deadline(delay_ms,accepted):
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01]);start=now[0]
    e.moveit.row=extended(target(start,1,None,start+50_000_000,e.session.sequence))
    e.tick();assert e.queued_moveit
    now[0]=start+delay_ms*1_000_000;e.tick()
    assert bool([c for c in d.calls if c[0]=='ServoJ'])==accepted
    assert (e.session.input_paused_ns is None)==accepted


@pytest.mark.parametrize('fault',['release','feedback','tracking'])
def test_extended_slot_does_not_delay_release_or_mask_live_fault(fault):
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    send(deadman=True,angular_rad_s=[0,0,.01]);start=now[0]
    e.moveit.row=extended(target(start,1,None,start+50_000_000,e.session.sequence))
    e.tick();now[0]=start+110_000_000
    if fault=='release':send()
    elif fault=='feedback':d.data['connected']=False
    else:e.queued_moveit['joints_rad'][5]=.1
    e.tick()
    assert not any(c[0]=='ServoJ' for c in d.calls)
    assert e.queued_moveit is None


def test_stream_grace_is_bounded_and_not_a_periodic_retransmission():
    e,d,send,_,now=setup(moveit_bundle='fixture');e.moveit=PacedBridge();e.moveit_api='cpp'
    e.moveit.info['delivery_timing']=dict(PROFILE)
    send(deadman=True,angular_rad_s=[0,0,.01]);start=now[0]
    e.moveit.row=extended(target(start,1,None,start,e.session.sequence));e.tick()
    now[0]=start+180_000_000;send(deadman=True,angular_rad_s=[0,0,.01]);e.tick()
    assert e.session.input_paused_ns is None and [c[0] for c in d.calls]==['ServoJ']
    now[0]=start+201_000_000;e.tick()
    assert e.session.input_paused_ns is not None and [c[0] for c in d.calls]==['ServoJ','Stop']


def test_extended_arm_certificate_still_requires_current_gripper_in_swept_range():
    now=[1_000_000_000]
    b=MoveItBridge('.',None,clock=lambda:now[0],api='cpp')
    r=extended(target(now[0],1,None,now[0],1))
    r['cpp']['gripper_check']=dict(checked_ns=now[0],arm_safe=True,fallback=False,arm_range_raw=[51,53])
    grip=dict(valid=True,actual=52/255,sent_raw=52,inflight_raw=None)
    b.gripper_snapshot=lambda:dict(grip,received_ns=now[0])
    now[0]+=120_000_000
    assert b.gripper_allows_arm(r)
    grip['sent_raw']=80
    assert not b.gripper_allows_arm(r)
    grip['sent_raw']=52;now[0]+=30_000_000
    assert not b.gripper_allows_arm(r)
