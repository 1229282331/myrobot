"""A delayed Stop reply must not rewrite an already emitted request event."""
from copy import deepcopy
import json
import pytest
from test_gateway import setup


@pytest.mark.parametrize('reply',[
    dict(acknowledged=True,accepted=True,reply='0,{},Stop();',reply_ns=42),
    dict(uncertain=True,error='reply_timeout',failed_ns=42),
])
def test_stop_request_is_identical_for_audits_draining_before_and_after_reply(reply):
    executor,device,_,_,_=setup()
    device.auto_ack=False
    executor.moving=True
    executor.request_stop()
    event=next(x for x in executor.events if x['event']=='stop_requested')
    expected=deepcopy(event)
    continuous=json.dumps(event)
    executor.stop_pending.event.update(reply)
    episode=json.dumps(event)
    assert continuous==episode
    assert event==expected
    assert not event['receipt']['acknowledged']
    assert executor.stop_pending.event!=event['receipt']


def test_stop_result_still_reports_the_real_later_reply():
    executor,device,_,step,_=setup()
    device.auto_ack=False
    executor.moving=True
    executor.request_stop()
    pending=executor.stop_pending
    pending.event.update(acknowledged=True,accepted=True,reply='0,{},Stop();',reply_ns=42)
    pending.done.set()
    step()
    requested=next(x for x in executor.events if x['event']=='stop_requested')
    result=next(x for x in executor.events if x['event']=='stop_result')
    assert not requested['receipt']['acknowledged']
    assert result['receipt']['acknowledged'] and result['receipt']['accepted']
