import asyncio
import json
import pytest
from e6_phosphobot.input_relay_timing import InputRelayTiming, relay_inputs


class Page:
    def __init__(self, messages):
        self.messages = messages

    async def iter_text(self):
        for raw in self.messages:
            yield raw


def test_relay_preserves_exact_bytes_and_order_without_recording_payloads():
    messages = [' {"seq": 1,"client_sample_ms":2.5,"private":"do not retain"} ',
                '{"seq":2,"client_sample_ms":3.5}', 'malformed']
    sent = []
    class Remote:
        async def send(self, raw):
            sent.append(raw)
    timing = InputRelayTiming(clock=iter(range(9)).__next__)
    asyncio.run(relay_inputs(Page(messages), Remote(), timing, 'one'))
    assert sent == messages
    result = timing.snapshot()
    assert [r['pc_received_ns'] for r in result['rows']] == [0,3,6]
    assert [r['pc_send_started_ns'] for r in result['rows']] == [1,4,7]
    assert [r['pc_send_finished_ns'] for r in result['rows']] == [2,5,8]
    assert [r.get('sequence') for r in result['rows']] == [1,2,None]
    assert all(r['outcome'] == 'sent' for r in result['rows'])
    assert 'do not retain' not in json.dumps(result)
    assert 'private' not in json.dumps(result)


@pytest.mark.parametrize('error', [RuntimeError('send failed'), asyncio.CancelledError()])
def test_failed_or_cancelled_send_is_retained_and_never_retried(error):
    attempts = []
    class Remote:
        async def send(self, raw):
            attempts.append(raw)
            raise error
    timing = InputRelayTiming(clock=iter(range(3)).__next__)
    with pytest.raises(type(error)):
        asyncio.run(relay_inputs(Page(['{"seq":1}', '{"seq":2}']), Remote(), timing, 'one'))
    assert attempts == ['{"seq":1}']
    assert timing.snapshot()['rows'][0]['outcome'] == 'interrupted'


@pytest.mark.parametrize('raw', ['[]', 'null', '{"seq":true,"client_sample_ms":NaN}',
                                '{"seq":{},"client_sample_ms":"private"}',
                                'x'*16385, '['*1500, json.dumps(dict(client_sample_ms=10**1000))])
def test_bad_metadata_cannot_interrupt_forwarding_or_leak_payloads(raw):
    timing = InputRelayTiming()
    timing.add('one', raw, 1, 2, 3, 'sent')
    row = timing.snapshot()['rows'][0]
    assert 'sequence' not in row and 'client_sample_ms' not in row


def test_ring_is_bounded_across_connections_and_reports_overwrites():
    timing = InputRelayTiming(capacity=2)
    for i in range(4):
        timing.add(str(i), '{}', i, i, i, 'sent')
    result = timing.snapshot()
    assert result['total'] == 4 and result['overwritten'] == 2
    assert [r['connection_id'] for r in result['rows']] == ['2','3']
