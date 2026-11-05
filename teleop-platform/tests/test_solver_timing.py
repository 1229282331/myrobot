"""Request slack must not change motion time, source age or release semantics."""
from copy import deepcopy

import pytest

from e6_phosphobot.cpp_cycle import CppCycle
from test_cpp_cycle import context, response


def ready(callback=55, request=70, paced=False):
    cycle=CppCycle(callback_budget_ms=callback,request_timeout_ms=request,writer_paced=paced)
    now=1_000_000_000
    cycle.update(context(now,active=False))
    assert cycle.request(now)
    cycle.complete(response(cycle),now+1_000_000)
    now+=2_000_000
    ctx=context(now)
    if paced:ctx['dispatch']=dict(version=1,receipt=None)
    cycle.update(ctx)
    assert cycle.request(now)
    return cycle,now


@pytest.mark.parametrize('callback,timeout_ms,elapsed,native_us,accepted',[
    (42,50,49,41999,True),(42,50,50,41999,False),
    (45,60,57,44999,True),(45,60,60,44999,False),
    (55,70,66,54999,True),(55,70,70,54999,False),
    (55,70,66,55000,False),(55,70,66,float('nan'),False),
])
def test_request_and_callback_deadlines_are_independent(callback,timeout_ms,elapsed,native_us,accepted):
    cycle,now=ready(callback,timeout_ms)
    reply=response(cycle);reply['callback_us']=native_us
    result=cycle.complete(reply,now+elapsed*1_000_000)
    assert (result is not None)==accepted
    if accepted:
        assert result['cpp']['period_s']==.05
        assert result['valid_until_ns']==now+100_000_000
    else:
        assert cycle.latest is None


@pytest.mark.parametrize('callback,timeout_ms',[(0,50),(55,55),(70,60),(55,101),(True,60),(45,60.),(45,float('inf'))])
def test_invalid_timing_is_rejected(callback,timeout_ms):
    with pytest.raises(ValueError,match='solver timing'):
        CppCycle(callback_budget_ms=callback,request_timeout_ms=timeout_ms)


@pytest.mark.parametrize('source', ['actual','command'])
def test_extended_compute_cannot_refresh_original_source_deadline(source):
    cycle,now=ready()
    # The source was already old when computation began; newer updates do not
    # change the pending request's evidence or its deadline.
    age=40_000_000 if source=='actual' else 190_000_000
    cycle.pending[source]['received_ns']-=age
    cycle.update(context(now+65_000_000))
    assert cycle.complete(response(cycle),now+65_000_000) is None
    assert cycle.info['last_rejection']=='solver_deadline'


@pytest.mark.parametrize('change',['release','epoch','generation'])
def test_extended_reply_after_release_or_context_change_is_discarded(change):
    cycle,now=ready()
    reply=response(cycle)
    ctx=context(now+60_000_000,active=change!='release',
                epoch=2 if change=='epoch' else 1,generation=2 if change=='generation' else 1)
    cycle.update(ctx)
    assert cycle.complete(reply,now+65_000_000) is None
    assert cycle.latest is None and cycle.reset_required


def test_slow_reply_waits_for_writer_parent_and_does_not_catch_up():
    cycle,now=ready(paced=True)
    reply=response(cycle)
    # Many updates during computation coalesce to the latest operator input.
    for i in range(1,7):
        ctx=context(now+i*10_000_000)
        ctx['command']['sequence']=10+i
        ctx['command']['intent']['joint_rad_s'][-1]=-.01
        ctx['dispatch']=dict(version=1,receipt=None)
        cycle.update(ctx)
        assert cycle.request(now+i*10_000_000) is None
    row=cycle.complete(reply,now+65_000_000)
    assert row and row['source_input']['sequence']==10
    assert cycle.request(now+66_000_000) is None
    receipt=dict(id=row['id'],epoch=row['epoch'],joints_rad=row['joints_rad'],sent_ns=now+67_000_000)
    ctx=deepcopy(cycle.context);ctx['dispatch']['receipt']=receipt
    cycle.update(ctx)
    assert cycle.request(now+68_000_000)
    assert cycle.pending['command']['sequence']==16
    assert cycle.pending['effective_intent']['joint_rad_s'][-1]==-.01
    assert cycle.pending['dispatch']['not_before_ns']==receipt['sent_ns']+50_000_000
    next_row=cycle.complete(response(cycle),now+69_000_000)
    assert next_row['cpp']['parent_id']==row['id']
    assert next_row['cpp']['period_s']==.05
    assert cycle.request(now+70_000_000) is None


def test_short_computation_still_obeys_nominal_twenty_hz():
    cycle,now=ready()
    assert cycle.complete(response(cycle),now+1_000_000)
    cycle.update(context(now+49_000_000))
    assert cycle.request(now+49_000_000) is None
    assert cycle.request(now+50_000_000)
