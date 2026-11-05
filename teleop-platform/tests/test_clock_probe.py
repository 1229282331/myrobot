import pytest
from e6_phosphobot.clock_probe import ClockProbes, input_latency_bound


def exchange():
    probes=ClockProbes()
    issued=probes.issue(100,1_110_000_000,1_112_000_000)
    response=dict(issued,client_received_ms=124)
    return probes,response


def test_four_timestamp_interval_matches_issued_exchange_and_is_single_use():
    probes,response=exchange()
    result=probes.complete(response,1_130_000_000)
    assert result['offset_lower_ms']==988 and result['offset_upper_ms']==1010
    assert result['network_rtt_ms']==22
    assert probes.complete(response,1_140_000_000) is None
    # True offset 1000 ms: browser input 200 -> X5 movement 1380 = 180 ms.
    bounds=input_latency_bound(200,1_380_000_000,result)
    assert bounds['lower_ms']<180<bounds['upper_ms']
    assert bounds['clock_width_ms']>22
    assert input_latency_bound(3200,4_380_000_000,result) is None


@pytest.mark.parametrize('field,value',[
    ('x5_received_ns',1),('x5_sent_ns',1),('client_received_ms',99),
    ('client_received_ms',float('nan')),('client_received_ms',float('inf')),
    ('client_received_ms',True),('client_received_ms',101),('client_sent_ms',99)])
def test_mismatched_or_impossible_clock_sample_is_not_used(field,value):
    probes,response=exchange();response[field]=value
    assert probes.complete(response,1_130_000_000) is None


def test_probe_expiry_and_memory_bound():
    probes,response=exchange()
    assert probes.complete(response,4_112_000_001) is None
    for i in range(20):probes.issue(i,1_110_000_000,1_112_000_000)
    assert len(probes.pending)==8
    for value in (None,True,-1,[],float('inf')):
        assert probes.issue(value,1,2) is None
