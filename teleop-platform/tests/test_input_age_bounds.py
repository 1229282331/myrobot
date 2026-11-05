import pytest

from e6_phosphobot.clock_probe import ClockProbes, InputAgeBounds
from e6_phosphobot.intent import IntentSession


def calibrated(a=1000, uplink=10, downlink=10):
    probe = ClockProbes()
    stamp = int((a+1000+uplink)*1e6)
    issued = probe.issue(a, stamp, stamp)
    return probe.complete(dict(issued, client_received_ms=a+uplink+downlink),
                          stamp+int((downlink+uplink)*1e6))


def msg(seq, sampled, echo=0, deadman=True, op='intent'):
    return dict(op=op, seq=seq, echo_ns=echo, client_sample_ms=sampled,
                source='keyboard', intent=dict(linear_m_s=[.002,0,0] if deadman else [0,0,0],
                angular_rad_s=[0]*3, joint_rad_s=[0]*6, gripper_rate_s=0,
                deadman=deadman, frame='base'))


def test_fresh_input_survives_delayed_state_without_extending_250ms_watchdog():
    now=[2_500_000_000];session=IntentSession(clock=lambda:now[0])
    session.observe_clock(calibrated())
    assert session.received_ns==0 and not session.ready
    assert session.receive(msg(1,1490,deadman=False))
    assert session.receive(msg(2,1490,deadman=False,op='prepare'))
    # The state round trip is old, while this input was sampled 10 ms ago.
    session.issued_ticks.add(2_100_000_000)
    assert session.receive(msg(3,1490,echo=2_100_000_000))
    assert session.last_freshness['source']=='clock_bound'
    assert session.active() is not None
    accepted=session.received_ns
    now[0]+=251_000_000
    session.observe_clock(calibrated(a=1600))
    assert session.received_ns==accepted
    assert session.active() is None and session.ready and session.input_paused_ns is not None
    # A fresh packet cannot silently prepare after a genuine input loss.
    assert not session.receive(msg(4,1740))
    assert session.ready and session.active() is None
    now[0]+=1_000_000
    assert session.receive(msg(5,1741,echo=session.clock_tick(),deadman=False))
    assert session.input_paused_ns is None and session.active() is None


def test_old_input_cannot_be_rejuvenated_by_clock_calibration():
    session=IntentSession(clock=lambda:2_500_000_000)
    session.observe_clock(calibrated())
    assert not session.receive(msg(1,1200))  # Actual input age 300 ms.
    assert session.received_ns==0 and session.intent is None
    assert session.last_freshness['source']=='expired'


def test_clock_expiry_and_missing_sync_keep_authenticated_echo_fallback():
    now=[5_100_000_000];session=IntentSession(clock=lambda:now[0])
    session.observe_clock(calibrated())
    assert not session.receive(msg(1,4095))
    assert not session.receive(msg(2,4095,echo=now[0])) # Not issued by owner.
    assert session.receive(msg(3,4095,echo=session.clock_tick()))
    assert session.last_freshness['source']=='echo'


def test_wide_or_future_exchange_cannot_replace_narrow_valid_proof():
    bounds=InputAgeBounds();bounds.add(calibrated())
    expected=bounds.bound(1490,2_500_000_000)
    bounds.add(calibrated(a=1100,uplink=10,downlink=300))
    assert bounds.bound(1490,2_500_000_000)==expected
    assert bounds.bound(900,2_500_000_000) is None
    assert bounds.bound(10000,2_500_000_000) is None
    for i in range(20):bounds.add(calibrated(a=1000+i))
    assert len(bounds.samples)==8


@pytest.mark.parametrize('uplink,downlink',[(1,1),(5,200),(200,5),(150,150)])
@pytest.mark.parametrize('input_age_ms',[0,20,249,251,400])
def test_bound_contains_true_age_for_asymmetric_network(uplink,downlink,input_age_ms):
    sample=calibrated(uplink=uplink,downlink=downlink)
    bounds=InputAgeBounds();bounds.add(sample)
    sampled=1600
    proof=bounds.bound(sampled,int((sampled+1000+input_age_ms)*1e6))
    assert proof['lower_ms'] <= input_age_ms <= proof['upper_ms']
    if input_age_ms>250:assert proof['upper_ms']>250
